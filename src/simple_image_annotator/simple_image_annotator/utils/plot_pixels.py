import matplotlib.pyplot as plt
import numpy as np

pixels_to_plot = [[115, 138, 157], [115, 138, 157], [115, 138, 157], [114, 137, 156], [115, 138, 157], [114, 137, 156], [114, 137, 156], [114, 137, 156], [115, 138, 157], [114, 137, 156]]


#
# DO NOT MODIFY
# Convert raw pixel values to NumPy array
num_pixels = len(pixels_to_plot)
img_array = np.array(pixels_to_plot, dtype=np.uint8).reshape((1, num_pixels, 3))  # shape: (1, num_pixels, 3)

# Display
H, W = 720, 1280  # full image width and height
plt.figure(figsize=(num_pixels, 2))  
plt.imshow(img_array, origin='lower', extent=(0, num_pixels, 0, 1))
plt.title("Middle 10 Pixels (Bottom Row)")
h, w, _ = img_array.shape
row = H - 1  

ax = plt.gca()
ax.set_xticks(np.arange(w) + 0.5)
ax.set_yticks([0.5])
ax.set_xticklabels([str(x + W // 2) for x in range(w)])
ax.set_yticklabels([str(row)])
for x in range(w + 1):
    ax.axvline(x=x, color='gray', linewidth=0.8)
for x in range(w):
    r, g, b = img_array[0, x]
    lum = 0.299 * r + 0.587 * g + 0.114 * b
    color = "black" if lum > 128 else "white"
    ax.text(x + 0.5, 0.5, f"({x + W // 2},{row})", ha="center", va="center",
            color=color, fontsize=8, fontweight="bold")

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_xlim(0, w)
ax.set_ylim(0, 1)
plt.tight_layout()
plt.show()

