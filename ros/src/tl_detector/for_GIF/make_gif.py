from PIL import Image
import glob
import os

files = sorted(glob.glob('./imgs_rosbag/*.jpg'),key=os.path.getmtime)
#files = sorted(glob.glob('./imgs_sim/*.jpg'),key=os.path.getmtime)
print(files)
images = list(map(lambda file: Image.open(file), files))

images[0].save('out.gif', save_all=True, append_images=images[1:], duration=100, loop=0)