#!/usr/bin/env python3
"""
Feature Detection Debugger
Shows exactly what ORB-SLAM3 "sees" and why initialization might fail

Usage: python3 debug_feature_detection.py --camera_id 0
"""

import cv2
import numpy as np
import argparse
import time


def analyze_frame(frame, orb):
    """Analyze a frame for SLAM suitability"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect ORB keypoints
    keypoints = orb.detect(gray, None)
    
    # Image quality metrics
    brightness = np.mean(gray)
    contrast = np.std(gray)
    laplacian = cv2.Laplacian(gray, cv2.CV_64F)
    sharpness = laplacian.var()
    
    # Check for motion blur
    blur_score = cv2.Laplacian(gray, cv2.CV_64F).var()
    
    return {
        'keypoints': keypoints,
        'num_features': len(keypoints),
        'brightness': brightness,
        'contrast': contrast,
        'sharpness': sharpness,
        'blur_score': blur_score,
        'gray': gray
    }


def draw_analysis(frame, analysis, show_features=True):
    """Draw analysis overlay on frame"""
    display = frame.copy()
    h, w = display.shape[:2]
    
    # Draw keypoints
    if show_features and len(analysis['keypoints']) > 0:
        cv2.drawKeypoints(display, analysis['keypoints'], display, 
                         color=(0, 255, 0), 
                         flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    # Status determination
    num_features = analysis['num_features']
    if num_features < 100:
        status = "CRITICAL: Too few features!"
        status_color = (0, 0, 255)  # Red
    elif num_features < 300:
        status = "WARNING: Few features"
        status_color = (0, 165, 255)  # Orange
    elif num_features < 500:
        status = "OK: Moderate features"
        status_color = (0, 255, 255)  # Yellow
    else:
        status = "EXCELLENT: Many features"
        status_color = (0, 255, 0)  # Green
    
    # Draw semi-transparent overlay for text background
    overlay = display.copy()
    cv2.rectangle(overlay, (0, 0), (w, 200), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, display, 0.4, 0, display)
    
    # Display metrics
    y_offset = 30
    cv2.putText(display, status, (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
    y_offset += 35
    
    metrics = [
        f"Features: {num_features} (need 500+)",
        f"Brightness: {analysis['brightness']:.1f} (optimal: 100-150)",
        f"Contrast: {analysis['contrast']:.1f} (need 30+)",
        f"Sharpness: {analysis['sharpness']:.1f} (need 100+)",
    ]
    
    for metric in metrics:
        cv2.putText(display, metric, (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += 25
    
    # Recommendations
    cv2.putText(display, "Press 'h' for help", (10, h - 15),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    
    return display


def show_help():
    """Print help message"""
    help_text = """
    ========================================
    FEATURE DETECTION DEBUGGER - HELP
    ========================================
    
    UNDERSTANDING THE DISPLAY:
    - Green circles = ORB features (corners, edges)
    - More features = Better for SLAM
    - Need 500+ features for good initialization
    
    FEATURE COUNT STATUS:
    - CRITICAL (< 100): SLAM will likely fail
    - WARNING (< 300): May struggle to initialize
    - OK (< 500): Should work but may be slow
    - EXCELLENT (500+): Optimal for SLAM
    
    WHAT TO DO IF FEW FEATURES:
    1. Add texture to environment:
       - Posters, pictures, patterns
       - Books, shelves with items
       - Textured objects
    
    2. Improve lighting:
       - Turn on more lights
       - Avoid glare/reflections
       - Even lighting is best
    
    3. Avoid:
       - Blank walls/ceilings
       - Uniform surfaces
       - Very dark/bright areas
       - Glass/mirrors
    
    4. Camera position:
       - 1-2 meters from surfaces
       - Angled view (not perpendicular)
       - Include edges/corners
    
    KEYBOARD CONTROLS:
    - 'h': Show this help
    - 'f': Toggle feature display
    - 's': Save current frame
    - 'q': Quit
    - '+/-': Adjust ORB threshold
    
    ========================================
    """
    print(help_text)


def main():
    parser = argparse.ArgumentParser(description='Feature Detection Debugger')
    parser.add_argument('--camera_id', type=int, default=0, help='Camera ID')
    parser.add_argument('--width', type=int, default=640, help='Frame width')
    parser.add_argument('--height', type=int, default=480, help='Frame height')
    args = parser.parse_args()
    
    # Open camera
    cap = cv2.VideoCapture(args.camera_id)
    if not cap.isOpened():
        print(f"Error: Cannot open camera {args.camera_id}")
        return
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    
    print("\n" + "="*60)
    print("ORB-SLAM3 Feature Detection Debugger")
    print("="*60)
    print("This tool shows what ORB-SLAM3 'sees'")
    print("Move camera to different areas to find best spots")
    print("Press 'h' for help, 'q' to quit")
    print("="*60 + "\n")
    
    # Create ORB detector (same settings as SLAM)
    fast_threshold = 20
    orb = cv2.ORB_create(
        nfeatures=1000,
        scaleFactor=1.2,
        nlevels=8,
        edgeThreshold=31,
        firstLevel=0,
        WTA_K=2,
        scoreType=cv2.ORB_HARRIS_SCORE,
        patchSize=31,
        fastThreshold=fast_threshold
    )
    
    show_features = True
    cv2.namedWindow('Feature Detection Debug', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Feature Detection Debug', 800, 600)
    
    # Statistics tracking
    feature_counts = []
    best_frame = None
    best_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            break
        
        # Analyze frame
        analysis = analyze_frame(frame, orb)
        feature_counts.append(analysis['num_features'])
        if len(feature_counts) > 30:
            feature_counts.pop(0)
        
        # Track best frame
        if analysis['num_features'] > best_count:
            best_count = analysis['num_features']
            best_frame = frame.copy()
        
        # Draw analysis
        display = draw_analysis(frame, analysis, show_features)
        
        # Add running average
        if len(feature_counts) > 0:
            avg_features = np.mean(feature_counts)
            cv2.putText(display, f"Avg: {avg_features:.0f} | Best: {best_count}",
                       (10, display.shape[0] - 45),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        cv2.imshow('Feature Detection Debug', display)
        
        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('h'):
            show_help()
        elif key == ord('f'):
            show_features = not show_features
            print(f"Feature display: {'ON' if show_features else 'OFF'}")
        elif key == ord('s'):
            timestamp = int(time.time())
            filename = f"debug_frame_{timestamp}.png"
            cv2.imwrite(filename, display)
            print(f"Saved: {filename}")
        elif key == ord('+') or key == ord('='):
            fast_threshold = max(5, fast_threshold - 2)
            orb = cv2.ORB_create(
                nfeatures=1000, scaleFactor=1.2, nlevels=8,
                fastThreshold=fast_threshold
            )
            print(f"FAST threshold: {fast_threshold} (lower = more features)")
        elif key == ord('-') or key == ord('_'):
            fast_threshold = min(30, fast_threshold + 2)
            orb = cv2.ORB_create(
                nfeatures=1000, scaleFactor=1.2, nlevels=8,
                fastThreshold=fast_threshold
            )
            print(f"FAST threshold: {fast_threshold} (higher = fewer features)")
    
    # Summary
    print("\n" + "="*60)
    print("SESSION SUMMARY")
    print("="*60)
    if len(feature_counts) > 0:
        print(f"Average features: {np.mean(feature_counts):.0f}")
        print(f"Best features: {best_count}")
        print(f"Min features: {np.min(feature_counts):.0f}")
        print(f"Max features: {np.max(feature_counts):.0f}")
        
        if best_count > 500:
            print("\n✓ GOOD: Found positions with sufficient features")
            print("  Use similar camera positions for SLAM")
        elif best_count > 300:
            print("\n⚠ MARGINAL: Found some features but not ideal")
            print("  SLAM may work but could be unstable")
        else:
            print("\n✗ POOR: Insufficient features detected")
            print("  SLAM initialization will likely fail")
            print("  Add more textured objects to environment")
    
    if best_frame is not None and best_count > 300:
        cv2.imwrite('best_frame.png', best_frame)
        print(f"\nSaved best frame with {best_count} features: best_frame.png")
    
    print("="*60)
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
