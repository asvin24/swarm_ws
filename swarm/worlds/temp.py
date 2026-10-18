from PIL import Image, ImageDraw

# Image size (1.2x1.2 meters, in pixels)
image_size = (1200, 1200)  # 1 pixel = 1 mm

# Create a white background image
image = Image.new("RGB", image_size, (255, 255, 255))  # White background

# Draw a red circle in the center
draw = ImageDraw.Draw(image)
circle_radius = 500  # 1 meter diameter -> 0.5 meter radius -> 500 pixels
circle_center = (image_size[0] // 2, image_size[1] // 2)

# Draw the circle
draw.ellipse(
    [
        (circle_center[0] - circle_radius, circle_center[1] - circle_radius),
        (circle_center[0] + circle_radius, circle_center[1] + circle_radius)
    ],
    fill=(255, 0, 0)  # Red color
)

# Save the image as a PNG
image.save("red_circle_marker.png")
