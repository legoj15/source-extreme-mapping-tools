import sys
import os

try:
    from PIL import Image
except ImportError:
    print("Error: PIL/Pillow not installed. Please run 'pip install pillow'")
    sys.exit(1)

def convert_tga_to_png(file_path):
    if not os.path.exists(file_path):
        return
    
    try:
        # Open the TGA file
        with Image.open(file_path) as img:
            # Construct png path
            base, ext = os.path.splitext(file_path)
            png_path = f"{base}.png"
            
            # Save as PNG
            img.save(png_path)
            print(f"Converted: {file_path} -> {png_path}")
            
    except Exception as e:
        print(f"Failed to convert {file_path}: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python tga2png.py <file1.tga> [file2.tga ...]")
        sys.exit(0)
        
    for arg in sys.argv[1:]:
        convert_tga_to_png(arg)
