#!/usr/bin/env python3
"""
generate_aruco_tags.py — Creates ArUco tag PNG images for Gazebo textures

HOW ARUCO TAGS WORK:
════════════════════
An ArUco tag is a synthetic binary marker — a square grid of black and white cells.
Each tag in a "dictionary" has a unique binary pattern that encodes an ID number.

Dictionary choice matters:
- DICT_4X4_50:  4×4 inner grid, 50 unique tags. Small grid = detectable at longer range
                but fewer unique IDs. Good for small-scale projects (< 50 tags).
- DICT_6X6_250: 6×6 grid, 250 tags. More IDs but needs to be closer/larger to detect.
- DICT_ARUCO_ORIGINAL: Classic 5×5, 1024 tags. Legacy compatibility.

For this project, DICT_4X4_50 is ideal: we only have 5 tags, and the 4×4 grid
is easier to detect from a flying drone (larger cells → more tolerant of blur).

USAGE:
  python3 generate_aruco_tags.py --output_dir ./tag_images --num_tags 5 --size 200

OUTPUT:
  Creates PNG files: aruco_0.png, aruco_1.png, ..., aruco_4.png
  These can be used as Gazebo model textures via <pbr><metal><albedo_map>
"""

import argparse
import os
import sys

try:
    import cv2
    from cv2 import aruco
except ImportError:
    print("ERROR: OpenCV with ArUco support is required.")
    print("Install with: pip install opencv-contrib-python")
    sys.exit(1)


def generate_tags(output_dir: str, num_tags: int, tag_size_px: int, dictionary_name: str):
    """
    Generate ArUco tag images.

    Args:
        output_dir: Directory to save PNG files
        num_tags: How many tags to generate (0 to num_tags-1)
        tag_size_px: Size of each tag image in pixels
        dictionary_name: ArUco dictionary to use
    """
    os.makedirs(output_dir, exist_ok=True)

    # Get the ArUco dictionary.
    # A dictionary defines the set of valid tag patterns.
    # Both the generator and detector MUST use the same dictionary,
    # or the detector won't recognize the tags.
    dict_lookup = {
        '4x4_50': aruco.DICT_4X4_50,
        '4x4_100': aruco.DICT_4X4_100,
        '5x5_50': aruco.DICT_5X5_50,
        '6x6_250': aruco.DICT_6X6_250,
    }

    if dictionary_name not in dict_lookup:
        print(f"Unknown dictionary: {dictionary_name}")
        print(f"Available: {list(dict_lookup.keys())}")
        sys.exit(1)

    aruco_dict = aruco.getPredefinedDictionary(dict_lookup[dictionary_name])

    for tag_id in range(num_tags):
        # generateImageMarker creates a binary image of the tag.
        # Args: dictionary, tag_id, output_size_px
        # Returns: grayscale numpy array (0=black, 255=white)
        tag_image = aruco.generateImageMarker(aruco_dict, tag_id, tag_size_px)

        # Add a white border around the tag.
        # ArUco detection REQUIRES a white border — without it,
        # the detector can't find the tag's outer boundary.
        border_size = tag_size_px // 4
        bordered = cv2.copyMakeBorder(
            tag_image,
            border_size, border_size, border_size, border_size,
            cv2.BORDER_CONSTANT, value=255
        )

        output_path = os.path.join(output_dir, f'aruco_{tag_id}.png')
        cv2.imwrite(output_path, bordered)
        print(f"Generated tag {tag_id} → {output_path} "
              f"({bordered.shape[1]}×{bordered.shape[0]} px)")

    print(f"\nDone! Generated {num_tags} tags using dictionary '{dictionary_name}'")
    print(f"IMPORTANT: Use the same dictionary ('{dictionary_name}') in your detector node!")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate ArUco tag images for Gazebo')
    parser.add_argument('--output_dir', default='./tag_images',
                        help='Output directory for PNG files')
    parser.add_argument('--num_tags', type=int, default=5,
                        help='Number of tags to generate')
    parser.add_argument('--size', type=int, default=200,
                        help='Tag size in pixels (before border)')
    parser.add_argument('--dictionary', default='4x4_50',
                        choices=['4x4_50', '4x4_100', '5x5_50', '6x6_250'],
                        help='ArUco dictionary to use')

    args = parser.parse_args()
    generate_tags(args.output_dir, args.num_tags, args.size, args.dictionary)
