import sys
import os
import cv2
import numpy as np

def compute_ssim(img1_gray, img2_gray):
    # Standard SSIM constants based on dynamic range of 255
    C1 = (0.01 * 255)**2
    C2 = (0.03 * 255)**2

    img1 = img1_gray.astype(np.float64)
    img2 = img2_gray.astype(np.float64)
    
    # 11x11 Gaussian kernel, sigma=1.5
    kernel = cv2.getGaussianKernel(11, 1.5)
    window = np.outer(kernel, kernel.transpose())

    mu1 = cv2.filter2D(img1, -1, window)[5:-5, 5:-5]
    mu2 = cv2.filter2D(img2, -1, window)[5:-5, 5:-5]
    mu1_sq = mu1**2
    mu2_sq = mu2**2
    mu1_mu2 = mu1 * mu2
    
    sigma1_sq = cv2.filter2D(img1**2, -1, window)[5:-5, 5:-5] - mu1_sq
    sigma2_sq = cv2.filter2D(img2**2, -1, window)[5:-5, 5:-5] - mu2_sq
    sigma12 = cv2.filter2D(img1 * img2, -1, window)[5:-5, 5:-5] - mu1_mu2

    ssim_map = ((2 * mu1_mu2 + C1) * (2 * sigma12 + C2)) / ((mu1_sq + mu2_sq + C1) * (sigma1_sq + sigma2_sq + C2))
    return ssim_map.mean()

def generate_diff_images(img1_color, img2_color, out_prefix):
    # Calculate absolute difference on grayscale to mimic old alpha representation
    img1_gray = cv2.cvtColor(img1_color, cv2.COLOR_BGR2GRAY)
    img2_gray = cv2.cvtColor(img2_color, cv2.COLOR_BGR2GRAY)
    
    abs_diff = cv2.absdiff(img1_gray, img2_gray)
    
    # Save base alpha diff
    alpha_path = f"{out_prefix}-alpha.png"
    cv2.imwrite(alpha_path, abs_diff)
    
    # Save contrast boosted diff (20x amplification)
    contrast_diff = cv2.convertScaleAbs(abs_diff, alpha=20.0, beta=0.0)
    contrast_path = f"{out_prefix}-alphacontrast.png"
    cv2.imwrite(contrast_path, contrast_diff)

from PIL import Image

def main():
    if len(sys.argv) != 4:
        print("Usage: python python_ssim_diff.py <image1.tga> <image2.tga> <out_prefix>")
        sys.exit(1)

    file1 = sys.argv[1]
    file2 = sys.argv[2]
    out_prefix = sys.argv[3]

    if not os.path.exists(file1) or not os.path.exists(file2):
        print(f"Error: Could not find inputs {file1} or {file2}")
        sys.exit(1)

    try:
        # Read with PIL as cv2.imread fails on Source Engine TGAs
        pil_img1 = Image.open(file1).convert('RGB')
        pil_img2 = Image.open(file2).convert('RGB')
        
        # Convert PIL image to OpenCV format (RGB -> BGR)
        img1_color = cv2.cvtColor(np.array(pil_img1), cv2.COLOR_RGB2BGR)
        img2_color = cv2.cvtColor(np.array(pil_img2), cv2.COLOR_RGB2BGR)
    except Exception as e:
        print(f"Error: Could not decode images - {e}")
        sys.exit(1)

    if img1_color is None or img2_color is None:
        print("Error: Could not decode images")
        sys.exit(1)

    if img1_color.shape != img2_color.shape:
        print(f"Error: Size mismatch {img1_color.shape} vs {img2_color.shape}")
        sys.exit(1)

    img1_gray = cv2.cvtColor(img1_color, cv2.COLOR_BGR2GRAY)
    img2_gray = cv2.cvtColor(img2_color, cv2.COLOR_BGR2GRAY)

    # Calculate SSIM
    s = compute_ssim(img1_gray, img2_gray)
    
    # Map 0.0 - 1.0 back to a difference percentage to match tgadiff output parser
    diff_percent = (1.0 - s) * 100.0
    print(f"Difference: {diff_percent:.4f}%")

    # Generate the visual differential PNGs directly without TGA intermediate
    generate_diff_images(img1_color, img2_color, out_prefix)

if __name__ == "__main__":
    main()
