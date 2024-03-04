import os
import cv2
import sys

def get_all_pngs(folder_path):
    filenames = os.listdir(folder_path)
    pngs = [filename for filename in filenames if filename.lower().endswith('.png')]
    full_pathnames = [os.path.join(folder_path, png) for png in pngs]
    return full_pathnames

def get_filename_without_extension(file_path):
    return os.path.splitext(os.path.basename(file_path))[0]

if __name__ == "__main__":
    # sys.argv[0] is the script name, and sys.argv[1:] contains the arguments
    args = sys.argv[1:]

    # Print the string arguments
    for arg in args:
        print("DIRNAME: ")
        print(arg)

        png_files = get_all_pngs(arg)

        for png_file in png_files:
            image = cv2.imread(png_file)
            new_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            filename_without_extension = get_filename_without_extension(png_file)+".png"

            cv2.imwrite(filename_without_extension, new_image)
            print(png_file)
