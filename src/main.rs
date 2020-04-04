/*
 * The MIT License
 *
 * Copyright 2020 sg4e.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

use scrap::*;
use std::fs::File;
use std::io::prelude::*;
use std::{thread, time};
use std::sync::{Arc, atomic::{Ordering, AtomicBool}};
use std::net::{TcpStream};
use std::sync::mpsc;
use std::io;
use std::io::ErrorKind::WouldBlock;
use std::process::exit;
use winapi::shared::windef::POINT;

const PIXEL_SIZE: usize = 4;
const BITMAP_FILE : &str = "test.bmp";
const SCREEN_REFRESH_RATE : usize = 60;
const CAPTURE_SAMPLE_RATE : usize = SCREEN_REFRESH_RATE * 2;
const CAPTURE_SAMPLE_DELAY_MILLIS : u64 = 1000 / CAPTURE_SAMPLE_RATE as u64;
const DELAY_MILLIS_AFTER_HOLE_SPLIT : u64 = 8 * 1000;

fn main() -> std::io::Result<()> {
    let version = match option_env!("APPVEYOR_BUILD_VERSION") {
        Some(val) => val,
        None => "dev"
    };
    let commit_hash = match option_env!("APPVEYOR_REPO_COMMIT") {
        Some(val) => val,
        None => "SNAPSHOT"
    };
    let datetime_of_build = match option_env!("APPVEYOR_REPO_COMMIT_TIMESTAMP") {
        Some(val) => val,
        None => "now"
    };
    println!("Waialae Autosplitter v{}-{}:{}", version, commit_hash, datetime_of_build);
    let input: UserInput = loop {
        let mut all_displays = Display::all().unwrap();
        let mut display_index: usize = 0;
        if all_displays.len() > 1 {
            display_index = loop {
                println!("You have a multi-monitor setup. Enter the number associated with the monitor \
                from which you want to capture gamefeed:");
                for (display_number, display) in all_displays.iter().enumerate() {
                    println!("{}: {}px x {}px", display_number + 1, display.width(), display.height());
                }
                let response = readline_stdin();
                match response.parse::<usize>() {
                    Ok(number) => {
                        if number <= all_displays.len() {
                            break number - 1; // user's list starts at 1 not 0
                        } else {
                            println!("Value {} is out of range. Try again", number);
                        }
                    }
                    Err(_) => println!("{} is not an acceptable value. Try again", response)
                }
            };
        }
        //get 2 mouse points for capture bounds
        let point_top_left = prompt_mouse_position("Hover your mouse over the TOP-LEFT corner of the gamefeed capture and press [Enter]");
        let point_bottom_right = prompt_mouse_position("Hover your mouse over the BOTTOM-RIGHT corner of the gamefeed capture and press [Enter]");
        //verify the bounds are valid
        if point_top_left.x > point_bottom_right.x || point_top_left.y > point_bottom_right.y {
            println!("Bottom-right corner is above or to the left of top-right corner. Try again");
            continue;
        }
        //relativize bounds to specified display
        //this assumes the displays are ordered left to right and sequentially
        //this probably won't work on some multi-monitor setups, but I can't find anything in the winapi
        let mut top_x : usize = point_top_left.x as usize;
        let top_y : usize = point_top_left.y as usize;
        let mut bottom_x : usize = point_bottom_right.x as usize;
        let bottom_y : usize = point_bottom_right.y as usize;
        for index in 0..display_index {
            let width = all_displays[index].width();
            top_x -= width;
            bottom_x -= width;
        }
        let dimensions = Rectangle {top_x, top_y, bottom_x, bottom_y};
        let dis = all_displays.remove(display_index);
        let mut cap = Capturer::new(dis).unwrap();
        let width = cap.width();
        let mut user_said_yes : bool = false;
        loop {
            match cap.frame() {
                Ok(frame) => {
                    let mut bitmap_prefix_data: [u8; 138] = [
                        0x42, 0x4D,             // Signature 'BM'
                        0xaa, 0x00, 0x00, 0x00, // Size: 170 bytes
                        0x00, 0x00,             // Unused
                        0x00, 0x00,             // Unused
                        0x8a, 0x00, 0x00, 0x00, // Offset to image data

                        0x7c, 0x00, 0x00, 0x00, // DIB header size (124 bytes)
                        0x04, 0x00, 0x00, 0x00, // Width (4px)
                        0x02, 0x00, 0x00, 0x00, // Height (2px)
                        0x01, 0x00,             // Planes (1)
                        0x20, 0x00,             // Bits per pixel (32)
                        0x03, 0x00, 0x00, 0x00, // Format (bitfield = use bitfields | no compression)
                        0x20, 0x00, 0x00, 0x00, // Image raw size (32 bytes)
                        0x13, 0x0B, 0x00, 0x00, // Horizontal print resolution (2835 = 72dpi * 39.3701)
                        0x13, 0x0B, 0x00, 0x00, // Vertical print resolution (2835 = 72dpi * 39.3701)
                        0x00, 0x00, 0x00, 0x00, // Colors in palette (none)
                        0x00, 0x00, 0x00, 0x00, // Important colors (0 = all)
                        0x00, 0x00, 0xFF, 0x00, // R bitmask (00FF0000)
                        0x00, 0xFF, 0x00, 0x00, // G bitmask (0000FF00)
                        0xFF, 0x00, 0x00, 0x00, // B bitmask (000000FF)
                        0x00, 0x00, 0x00, 0x00, // A bitmask (FF000000)
                        0x42, 0x47, 0x52, 0x73, // sRGB color space
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Unused R, G, B entries for color space
                        0x00, 0x00, 0x00, 0x00, // Unused Gamma X entry for color space
                        0x00, 0x00, 0x00, 0x00, // Unused Gamma Y entry for color space
                        0x00, 0x00, 0x00, 0x00, // Unused Gamma Z entry for color space

                        0x00, 0x00, 0x00, 0x00, // Unknown
                        0x00, 0x00, 0x00, 0x00, // Unknown
                        0x00, 0x00, 0x00, 0x00, // Unknown
                        0x00, 0x00, 0x00, 0x00, // Unknown

                        // Image data:
                        // 0xFF, 0x00, 0x00, 0x7F, // Bottom left pixel
                        // 0x00, 0xFF, 0x00, 0x7F,
                        // 0x00, 0x00, 0xFF, 0x7F,
                        // 0xFF, 0xFF, 0xFF, 0x7F, // Bottom right pixel
                        // 0xFF, 0x00, 0x00, 0xFF, // Top left pixel
                        // 0x00, 0xFF, 0x00, 0xFF,
                        // 0x00, 0x00, 0xFF, 0xFF,
                        // 0xFF, 0xFF, 0xFF, 0xFF  // Top right pixel
                    ];
                    { // do file I/O
                        let mut file = File::create(BITMAP_FILE)?;
                        let mut image = Image::from_byte_array(width, &*frame);
                        image.set_sub_frame(dimensions);

                        write_le_u32(18, image.width as u32, &mut bitmap_prefix_data);
                        write_le_u32(22, image.height as u32, &mut bitmap_prefix_data);

                        let image_total_bytes = image.total_bytes();
                        write_le_u32(34, image_total_bytes as u32, &mut bitmap_prefix_data);
                        write_le_u32(2, (bitmap_prefix_data.len() + image_total_bytes) as u32, &mut bitmap_prefix_data);

                        file.write_all(&bitmap_prefix_data)?;
                        for row in image.get_visible_rows().iter().rev() {
                            file.write_all(row.to_byte_array())?;
                        }
                    }
                    print!("An image of the capture area has been printed to {}. \
                    Does this image encapsulate the entirety of the gamefeed and nothing else? [Y/n] ", BITMAP_FILE);
                    io::stdout().flush()?;
                    let response = readline_stdin();
                    if response.is_empty() || response.starts_with("y") {
                        user_said_yes = true;
                    }
                    break;
                },
                Err(e) => handle_capture_error(e)
            }
        }
        if user_said_yes {
            break UserInput {dimensions, cap};
        }
    };
    let mut cap = input.cap;
    let width = cap.width();

    //init thread for connection to LiveSplit.Server
    match TcpStream::connect("localhost:16834") {
        Ok(mut stream) => {
            let (livesplit_socket, rx) = mpsc::channel();
            thread::Builder::new().name("livesplit_socket_writer".to_string()).spawn(move || {
                loop {
                    let control: Control = rx.recv().unwrap();
                    //from LiveStream.Server readme: <command><space><parameters><\r\n>
                    let command = match control {
                        Control::Start => "starttimer",
                        Control::Split => "split",
                        Control::Reset => "reset",
                        Control::SkipSplit => "skipsplit",
                        Control::Unsplit => "unsplit",
                    };
                    stream.write_all(format!("{}\r\n", command).as_bytes()).unwrap_or_else(|error| {
                        println!("Error sending {} to LiveSplit: {}", command, error);
                    });
                    stream.flush().unwrap();
                }
            }).unwrap();
            //console reader thread
            let (main_control_writer, main_control_receiver) = mpsc::channel();
            let img_proc_main_control_handle = main_control_writer.clone();
            thread::Builder::new().name("console_reader".to_string()).spawn(move || {
                let stdin = io::stdin();
                loop {
                    let mut buffer : String = String::new();
                    //Rust doesn't seem to expose any raw access to stdin
                    //This means we can't get input until a newline, unfortunately
                    //The ncurses crate may be a workaround for this, and may also offer tools to create a better all-around CLI client
                    match stdin.read_line(&mut buffer) {
                        Ok(_) => {
                            let control: Control = match buffer.to_lowercase().trim() {
                                "" => Control::Split,
                                "r" => Control::Reset,
                                "s" => Control::SkipSplit,
                                "u" => Control::Unsplit,
                                _ => {
                                    println!("Bad command: {}", buffer);
                                    continue;
                                }
                            };
                            main_control_writer.send(control).unwrap();
                        },
                        Err(e) => println!("Error reading command-line input: {}", e)
                    }
                }
            }).unwrap();
            //Personally, I'd prefer to put the image processing as its own thread and reserve the main thread as the control thread
            //that processes all data the other threads send it. However, Rust requires that the Capture object not be sent to another thread.
            //Therefore, we make a control thread, and leave the image processing on the main thread.
            let image_processing_thread_handle = std::thread::current();
            let wake_up_flag_store = Arc::new(AtomicBool::new(false));
            let wake_up_flag_read = Arc::clone(&wake_up_flag_store);
            thread::Builder::new().name("logic_control".to_string()).spawn(move || {
                let mut hole = 0;
                let mut playing_back_nine_interlude : bool = false;
                loop {
                    //TODO console-input parsing during sleep
                    match main_control_receiver.recv() {
                        Ok(control) => {
                            match control {
                                Control::Reset => {
                                    //sleep image-proc thread
                                    wake_up_flag_store.store(false, Ordering::Release);
                                    livesplit_socket.send(control).unwrap();
                                    hole = 0;
                                    playing_back_nine_interlude = false;
                                    println!("Reset");
                                }
                                Control::Split | Control::Start => {
                                    //this manual split input is for final hole split
                                    //and for 1st split "when golfer becomes visible", until autosplit for 1st split is coded
                                    //it can also be used to manually split if, for whatever reason, the autosplit fails
                                    if hole < 19 {
                                        hole += 1;
                                    }
                                    if hole == 11 && playing_back_nine_interlude {
                                        //don't split in this case
                                        hole -= 1;
                                        playing_back_nine_interlude = false;
                                        //skip
                                    }
                                    else if hole > 18 {
                                        livesplit_socket.send(control).unwrap();
                                        println!("Done!");
                                        //sleep image-proc thread
                                        wake_up_flag_store.store(false, Ordering::Release);
                                    } 
                                    else {
                                        if hole == 1 {
                                            livesplit_socket.send(Control::Start).unwrap();
                                        }
                                        else {
                                            livesplit_socket.send(Control::Split).unwrap();
                                        }
                                        println!("Hole {}", hole);
                                        if hole == 1 {
                                            //wake image-proc thread
                                            wake_up_flag_store.store(true, Ordering::Release);
                                            image_processing_thread_handle.unpark();
                                        }
                                        else if hole == 10 {
                                            playing_back_nine_interlude = true;
                                        }
                                    }
                                },
                                Control::Unsplit => {
                                    livesplit_socket.send(control).unwrap();
                                    hole -= 1;
                                    println!("Hole {}", hole);
                                },
                                Control::SkipSplit => {
                                    livesplit_socket.send(control).unwrap();
                                    hole += 1;
                                    println!("Hole {}", hole);
                                }
                            }
                        },
                        Err(e) => {
                            println!("Error receiving input for control thread: {}", e); //this shouldn't happen
                        },
                    }
                }
            }).unwrap();
            loop {
                while !wake_up_flag_read.load(Ordering::Acquire) {
                    thread::park();
                }
                match cap.frame() {
                    Ok(frame) => {
                        let mut waialae_image = Image::from_byte_array(width, &*frame);
                        waialae_image.set_sub_frame(input.dimensions);
                        if waialae_image.is_black() {
                            img_proc_main_control_handle.send(Control::Split).unwrap();
                            thread::sleep(time::Duration::from_millis(DELAY_MILLIS_AFTER_HOLE_SPLIT));
                        } else {
                            thread::sleep(time::Duration::from_millis(CAPTURE_SAMPLE_DELAY_MILLIS));
                        }
                    }
                    Err(e) => handle_capture_error(e)
                }
            }
        }
        Err(e) => {
            println!("Failed to connect to LiveSplit Server: {}", e);
        }
    }

    Ok(())
}

fn handle_capture_error(e: std::io::Error) -> () {
    if e.kind() == WouldBlock {
        //println!("Waiting on frame...");
        thread::sleep(time::Duration::from_millis(CAPTURE_SAMPLE_DELAY_MILLIS));
    }
    else {
        println!("Unknown error during screen capture: {}. Exiting", e);
        exit(-1);
    }
}

fn prompt_mouse_position(prompt: &str) -> POINT {
    println!("{}", prompt);
    readline_stdin();
    get_mouse_position()
}

fn get_mouse_position() -> POINT {
    use winapi::um::winuser::GetCursorPos;
    let mut point = POINT { x: 0, y: 0 };
    unsafe { GetCursorPos(&mut point as *mut POINT); }
    point
}

fn readline_stdin() -> String {
    let mut response = String::new();
    io::stdin().read_line(&mut response).unwrap();
    response.trim().to_lowercase()
}

struct UserInput {
    dimensions: Rectangle,
    cap: Capturer,
}

enum Control {
    Start,
    Reset,
    Split,
    Unsplit,
    SkipSplit,
}

//LiveSplit server commands
// const START_TIMER: &str = "starttimer";
// const START_OR_SPLIT: &str = "startorsplit";
// const SPLIT: &str = "split";
// const UNSPLIT: &str = "unsplit";
// const SKIP_SPLIT: &str = "skipsplit";
// const PAUSE: &str = "pause";
// const RESUME: &str = "resume";
// const RESET: &str = "reset";

const BLUE_INDEX: usize = 0;
const GREEN_INDEX: usize = 1;
const RED_INDEX: usize = 2;
//Alpha = 3

#[derive(Clone, Copy, Debug)]
struct Rectangle {
    top_x: usize,
    top_y: usize,
    bottom_x: usize,
    bottom_y: usize
}

fn is_black(pixel: &[u8]) -> bool {
    const BLACK_THRESHOLD: u8 = 50;
    pixel[BLUE_INDEX] < BLACK_THRESHOLD &&
        pixel[GREEN_INDEX] < BLACK_THRESHOLD &&
        pixel[RED_INDEX] < BLACK_THRESHOLD
}

struct Row<'a> {
    pixels: &'a[u8],
    start: usize, //inclusive, in bytes, not pixels
    end: usize //exclusive, in bytes, not pixels
}

impl Row<'_> {
    fn from_byte_array(arr: &[u8]) -> Row {
        let row = Row {pixels: arr, start: 0, end: arr.len()};
        row
    }

    fn to_byte_array(&self) -> &[u8] {
        self.pixels[self.start..self.end].as_ref()
    }

    fn count_black_pixels(&self) -> usize {
        let mut count = 0usize;
        for pixel in self.pixels[self.start..self.end].chunks(PIXEL_SIZE) {
            if is_black(pixel) {
                count += 1;
            }
        }
        count
    }

    fn set_start(&mut self, start: usize) -> () {
        self.start = start * PIXEL_SIZE;
    }

    fn set_end(&mut self, end: usize) -> () {
        self.end = end * PIXEL_SIZE;
    }
}

struct Image<'a> {
    height: usize,
    width: usize,
    rows: Vec<Row<'a>>,
    sub_frame: Rectangle,
}

impl Image<'_> {
    fn from_byte_array(width: usize, arr: &[u8]) -> Image {
        let height = arr.len()/PIXEL_SIZE/width;
        let mut im = Image {height, width, rows: Vec::with_capacity(height),
            sub_frame: Rectangle {top_x: 0, top_y: 0, bottom_x: width, bottom_y: height} };
        for row in arr.chunks(im.width * PIXEL_SIZE) {
            im.rows.push(Row::from_byte_array(row));
        }
        im
    }

    fn is_black(&self) -> bool {
        let threshold = 95; //percent
        let pixel_count : usize = self.width * self.height;
        let trigger : usize = pixel_count * threshold / 100;
        let mut count : usize = 0;
        for i in self.sub_frame.top_y..self.sub_frame.bottom_y {
            let row = &self.rows[i];
            count += row.count_black_pixels();
            if count >= trigger {
                return true;
            }
        };
        false
    }

    fn get_visible_rows(&self) -> &[Row] {
        &self.rows[self.sub_frame.top_y..self.sub_frame.bottom_y]
    }

    fn set_sub_frame(&mut self, dim: Rectangle) -> () {
        for row in &mut self.rows {
            row.set_start(dim.top_x);
            row.set_end(dim.bottom_x);
        }
        self.height = dim.bottom_y - dim.top_y;
        self.width = dim.bottom_x - dim.top_x;
        self.sub_frame = dim;
    }

    fn total_bytes(&self) -> usize {
        self.height * self.width * PIXEL_SIZE
    }

}

fn write_le_u32(offset: usize, value: u32, arr: &mut [u8]) -> () {
    arr[offset + 0] = (value >> 0 & 0xFF) as u8;
    arr[offset + 1] = (value >> (1 * 8) & 0xFF) as u8;
    arr[offset + 2] = (value >> (2 * 8) & 0xFF) as u8;
    arr[offset + 3] = (value >> (3 * 8) & 0xFF) as u8;
}
