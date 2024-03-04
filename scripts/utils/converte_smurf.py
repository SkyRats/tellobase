import os
import cv2
import sys

if __name__ == "__main__":
    # sys.argv[0] is the script name, and sys.argv[1:] contains the arguments
    args = sys.argv[1:]

    # Print the string arguments
    for arg in args:
        print(arg)

def get_all_pngs(folder_path):
    filenames = os.listdir(folder_path)
    pngs = [filename for filename in filenames if filename.lower().endswith('.png')]
    full_pathnames = [os.path.join(folder_path, png) for png in pngs]
    return full_pathnames

if __name__ == "__main__":
    folder_path = "/home/software/Documents/tellobase/scripts/picture_game/spingame_9/"

    png_files = get_all_pngs(folder_path)

    for png_file in png_files:
        image = cv2.imread("/home/software/Documents/tellobase/scripts/picture_game/spingame_9/picture_0.png")
        print(png_file)




cv2.imshow("Image", image)

cv2.waitKey(0)

cv2.imshow("Image", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))

# Wait for the user to press a key
cv2.waitKey(0)
 
# Close all windows
cv2.destroyAllWindows()

