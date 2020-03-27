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

const BYTE_SIZE: usize = 4;
const BITMAP_FILE : &str = "test.bmp";

fn main() -> std::io::Result<()> {
    let mut cap : Capturer = loop {
        //TODO get mouse coordinates with Windows API
        let mut all_displays = Display::all().unwrap();
        // get 2nd
        //all_displays.pop();
        let dis = all_displays.pop().unwrap();
        let mut test_cap = Capturer::new(dis).unwrap();
        let width = test_cap.width();
        let height = test_cap.height();
        let frame = test_cap.frame().unwrap();
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
        write_le_u32(2, bitmap_prefix_data.len() + frame.len(), &mut bitmap_prefix_data);
        write_le_u32(18, width, &mut bitmap_prefix_data);
        write_le_u32(22, height, &mut bitmap_prefix_data);

        let mut file = File::create(BITMAP_FILE)?;
        let image = Image::from_byte_array(width, &*frame);
        let image_bytes: Vec<u8> = image.rows.iter().rev().map(|r| r.to_byte_array()).flatten().collect();
        write_le_u32(34, image_bytes.len(), &mut bitmap_prefix_data);
        file.write_all(&bitmap_prefix_data)?;
        file.write_all(image_bytes.as_ref())?;
        file.flush()?;
        print!("An image of the capture area has been printed to {}. \
        Does this image encapsulate the entirety of the gamefeed and nothing else? [Y/n] ", BITMAP_FILE);
        io::stdout().flush()?;
        let mut response = String::new();
        io::stdin().read_line(&mut response).unwrap();
        if response.trim().is_empty() || response.to_lowercase().starts_with("y") {
            break test_cap;
        }
    };

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
                    })
                }
            });
            loop {
                let width = cap.width();
                let screenshot = cap.frame()?;
                let waialae_image = Image::from_byte_array(width, &*screenshot);
                if waialae_image.is_black() {
                    livesplit_socket.send(SPLIT).unwrap();
                    thread::sleep(time::Duration::from_secs(6));
                } else {
                    thread::sleep(time::Duration::from_millis(1000 / 120));
                }
            }
        }
        Err(e) => {
            println!("Failed to connect to LiveSplit Server: {}", e);
        }
    }

    Ok(())
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

struct Pixel {
    // b: u8,
    // g: u8,
    // r: u8,
    // a: u8,
    data: Vec<u8>,
}

#[allow(dead_code)]
enum Color {
    Blue = 0,
    Green = 1,
    Red = 2,
    Alpha = 3
}

impl Pixel {

    fn get(&self, color: Color) -> u8 {
        self.data[color as usize]
    }

    fn is_black(&self) -> bool {
        const BLACK_THRESHOLD: u8 = 50;
        self.get(Color::Blue) < BLACK_THRESHOLD &&
            self.get(Color::Green) < BLACK_THRESHOLD &&
            self.get(Color::Red) < BLACK_THRESHOLD
    }
}

struct Row {
    pixels: Vec<Pixel>,
}

impl Row {
    fn from_byte_array(arr: &[u8]) -> Row {
        let mut row = Row {pixels: Vec::with_capacity(arr.len()/BYTE_SIZE)};
        for p in arr.chunks(BYTE_SIZE) {
            row.pixels.push(Pixel {data: p.to_vec()});
        }
        row
    }

    fn to_byte_array(&self) -> Vec<u8> {
        self.pixels.iter().map(|p| p.data.to_vec()).flatten().collect()
    }
}

#[allow(dead_code)]
struct Image {
    height: usize,
    width: usize,
    rows: Vec<Row>,
}

impl Image {
    fn from_byte_array(width: usize, arr: &[u8]) -> Image {
        let height = arr.len()/BYTE_SIZE/width;
        let mut im = Image {height, width, rows: Vec::with_capacity(height)};
        for row in arr.chunks(im.width * BYTE_SIZE) {
            im.rows.push(Row::from_byte_array(row));
        }
        im
    }

    fn is_black(&self) -> bool {
        let threshold = 95; //percent
        let pixel_count : usize = self.rows.iter().map(|row| row.pixels.len()).sum();
        let trigger : usize = pixel_count * threshold / 100;
        let mut count : usize = 0;
        for row in &self.rows {
            count += row.pixels.iter().filter(|pix| pix.is_black()).count();
            if count >= trigger {
                return true;
            }
        };
        false
    }
}

fn write_le_u32(offset: usize, value: usize, arr: &mut [u8]) -> () {
    arr[offset + 0] = (value >> 0 & 0xFF) as u8;
    arr[offset + 1] = (value >> (1 * 8) & 0xFF) as u8;
    arr[offset + 2] = (value >> (2 * 8) & 0xFF) as u8;
    arr[offset + 3] = (value >> (3 * 8) & 0xFF) as u8;
}
