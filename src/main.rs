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
use std::net::{TcpStream};
use std::sync::mpsc;
use std::io;
use std::io::ErrorKind::WouldBlock;
use std::process::exit;

const PIXEL_SIZE: usize = 4;
const BITMAP_FILE : &str = "test.bmp";
const SCREEN_REFRESH_RATE : usize = 60;
const CAPTURE_SAMPLE_RATE : usize = SCREEN_REFRESH_RATE * 2;
const CAPTURE_SAMPLE_DELAY_MILLIS : u64 = 1000 / CAPTURE_SAMPLE_RATE as u64;
const DELAY_MILLIS_AFTER_HOLE_SPLIT : u64 = 6 * 1000;

fn main() -> std::io::Result<()> {
    //TODO get mouse coordinates with Windows API
    let top_x = 80;
    let top_y = 80;
    let bottom_x = 800;
    let bottom_y = 800;
    let dimensions = Rectangle {top_x, top_y, bottom_x, bottom_y};

    let mut all_displays = Display::all().unwrap();
    // get 2nd
    //all_displays.pop();
    let dis = all_displays.pop().unwrap();
    let mut cap = Capturer::new(dis).unwrap();
    let width = cap.width();
    let input: UserInput = loop {
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
                    for row in image.rows.iter().rev() {
                        file.write_all(row.to_byte_array())?;
                    }
                }
                print!("An image of the capture area has been printed to {}. \
                Does this image encapsulate the entirety of the gamefeed and nothing else? [Y/n] ", BITMAP_FILE);
                io::stdout().flush()?;
                let mut response = String::new();
                io::stdin().read_line(&mut response).unwrap();
                if response.trim().is_empty() || response.to_lowercase().starts_with("y") {
                    break UserInput {dimensions, cap};
                }
            },
            Err(e) => handle_capture_error(e)
        }
    };
    let mut cap = input.cap;
    let width = cap.width();

    //init thread for connection to LiveSplit.Server
    match TcpStream::connect("localhost:16834") {
        Ok(mut stream) => {
            let (livesplit_socket, rx) = mpsc::channel();
            thread::spawn(move || {
                loop {
                    let command: &str = rx.recv().unwrap();
                    //from LiveStream.Server readme: <command><space><parameters><\r\n>
                    stream.write_all(format!("{} \r\n", command).as_bytes()).unwrap_or_else(|error| {
                        println!("Error sending {} to LiveSplit: {}", command, error);
                    });
                    stream.flush().unwrap();
                }
            });
            loop {
                match cap.frame() {
                    Ok(frame) => {
                        let mut waialae_image = Image::from_byte_array(width, &*frame);
                        waialae_image.set_sub_frame(input.dimensions.clone());
                        if waialae_image.is_black() {
                            livesplit_socket.send(SPLIT).unwrap();
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

struct UserInput {
    dimensions: Rectangle,
    cap: Capturer,
}

    //LiveSplit server commands
    #[allow(dead_code)]
    const START_TIMER: &str = "starttimer";
    #[allow(dead_code)]
    const START_OR_SPLIT: &str = "startorsplit";
    #[allow(dead_code)]
    const SPLIT: &str = "split";
    #[allow(dead_code)]
    const UNSPLIT: &str = "unsplit";
    #[allow(dead_code)]
    const SKIP_SPLIT: &str = "skipsplit";
    #[allow(dead_code)]
    const PAUSE: &str = "pause";
    #[allow(dead_code)]
    const RESUME: &str = "resume";
    #[allow(dead_code)]
    const RESET: &str = "reset";

const BLUE_INDEX: usize = 0;
const GREEN_INDEX: usize = 1;
const RED_INDEX: usize = 2;
//Alpha = 3

#[derive(Clone, Copy)]
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

    fn set_sub_frame(&mut self, dim: Rectangle) -> () {
        for row in &mut self.rows {
            row.set_start(dim.top_x);
            row.set_end(dim.bottom_y);
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
