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

use captrs::*;
use std::fs::File;
use std::io::prelude::*;
use std::{thread, time};
use std::net::{TcpStream};
use std::sync::mpsc;
use std::io;

const PIXEL_SIZE: usize = 4;
const BITMAP_FILE : &str = "test.bmp";

fn main() -> std::io::Result<()> {
    let input: UserInput = loop {
        //TODO get mouse coordinates with Windows API
        let top_x = 80;
        let top_y = 80;
        let bottom_x = 800;
        let bottom_y = 800;
        let dimensions = Rectangle {top_x, top_y, bottom_x, bottom_y};

        let mut test_cap = Capturer::new(0).unwrap();
        thread::sleep(time::Duration::from_millis(1000));
        test_cap.capture_store_frame().unwrap();
        let frame = test_cap.get_stored_frame().unwrap();
        let (width, height) = test_cap.geometry();
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
        let mut file = File::create(BITMAP_FILE)?;
        println!("{}", width);
        let mut image = Image::from_byte_array(width as usize, frame);
        image.set_sub_frame(dimensions.clone());

        write_le_u32(18, image.width as u32, &mut bitmap_prefix_data);
        write_le_u32(22, image.height as u32, &mut bitmap_prefix_data);

        let image_total_bytes = image.total_bytes();
        write_le_u32(34, image_total_bytes as u32, &mut bitmap_prefix_data);
        write_le_u32(2, (bitmap_prefix_data.len() + image_total_bytes) as u32, &mut bitmap_prefix_data);

        file.write_all(&bitmap_prefix_data)?;
        for row in image.get_visible_rows().iter().rev() {
            file.write_all(row.to_byte_array().as_slice())?;
        }
        file.flush()?;
        print!("An image of the capture area has been printed to {}. \
        Does this image encapsulate the entirety of the gamefeed and nothing else? [Y/n] ", BITMAP_FILE);
        io::stdout().flush()?;
        let mut response = String::new();
        io::stdin().read_line(&mut response).unwrap();
        if response.trim().is_empty() || response.to_lowercase().starts_with("y") {
            break UserInput {dimensions, cap: test_cap};
        }
    };
    let mut cap = input.cap;

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
                let (width, height) = cap.geometry();
                let cap_result = cap.capture_store_frame();
                if cap_result.is_err() {
                    println!("No screen update");
                    thread::sleep(time::Duration::from_millis(1000 / 60));
                    continue;
                }
                let screenshot = cap.get_stored_frame().unwrap();
                let mut waialae_image = Image::from_byte_array(width as usize, &screenshot);
                waialae_image.set_sub_frame(input.dimensions.clone());
                if waialae_image.is_black() {
                    livesplit_socket.send(SPLIT).unwrap();
                    thread::sleep(time::Duration::from_secs(6));
                } else {
                    thread::sleep(time::Duration::from_millis(1000 / 60));
                }
            }
        }
        Err(e) => {
            println!("Failed to connect to LiveSplit Server: {}", e);
        }
    }

    Ok(())
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

#[derive(Clone)]
struct Rectangle {
    top_x: usize,
    top_y: usize,
    bottom_x: usize,
    bottom_y: usize
}

fn is_black(pixel: &Bgr8) -> bool {
    const BLACK_THRESHOLD: u8 = 50;
    pixel.b < BLACK_THRESHOLD &&
        pixel.g < BLACK_THRESHOLD &&
        pixel.r < BLACK_THRESHOLD
}

// fn bgr8_to_bytes<'a>(bgr8: &'a Bgr8) -> &'a[u8] {
//     &[bgr8.b, bgr8.g, bgr8.r, bgr8.a]
// }

struct Row<'a> {
    pixels: &'a[Bgr8],
    start: usize, //inclusive, in pixels
    end: usize //exclusive, in pixels
}

impl Row<'_> {
    fn from_byte_array(arr: &[Bgr8]) -> Row {
        let row = Row {pixels: arr, start: 0, end: arr.len()};
        row
    }

    fn to_byte_array(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity((self.end - self.start) * PIXEL_SIZE);
        for pixel in self.pixels[self.start..self.end].iter() {
            bytes.push(pixel.b);
            bytes.push(pixel.g);
            bytes.push(pixel.r);
            bytes.push(pixel.a);
        }
        bytes
    }

    fn count_black_pixels(&self) -> usize {
        let mut count = 0usize;
        for pixel in self.pixels[self.start..self.end].iter() {
            if is_black(pixel) {
                count += 1;
            }
        }
        count
    }

    fn set_start(&mut self, start: usize) -> () {
        self.start = start;
    }

    fn set_end(&mut self, end: usize) -> () {
        self.end = end;
    }
}

struct Image<'a> {
    height: usize,
    width: usize,
    rows: Vec<Row<'a>>,
    sub_frame: Rectangle,
}

impl Image<'_> {
    fn from_byte_array(width: usize, arr: &[Bgr8]) -> Image {
        let height = arr.len()/width;
        let mut im = Image {height, width, rows: Vec::with_capacity(height),
            sub_frame: Rectangle {top_x: 0, top_y: 0, bottom_x: width, bottom_y: height} };
        for row in arr.chunks(im.width) {
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

    fn get_visible_rows<'a>(&'a self) -> &[Row] {
        self.rows[self.sub_frame.top_y..self.sub_frame.bottom_y].as_ref()
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
