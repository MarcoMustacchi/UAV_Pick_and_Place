from PIL import Image, ImageOps

# Load the marker image
marker = Image.open('aruco_mark_7.png')

# Define padding size
padding = 50  # Adjust as needed

# Add white padding
padded_marker = ImageOps.expand(marker, border=padding, fill='white')

# Save the padded marker
padded_marker.save('aruco_mark_7_White.png')
