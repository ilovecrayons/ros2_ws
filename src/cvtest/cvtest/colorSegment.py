import cv2
import numpy as np
import time

class ColorSegmenter:
    def __init__(self):
        self.color_ranges = {
            'red': [(0, 50, 50), (10, 255, 255), (170, 50, 50), (180, 255, 255)], 
            'blue': [(100, 50, 50), (130, 255, 255)],
            'green': [(40, 50, 50), (80, 255, 255)],
            'yellow': [(20, 50, 50), (40, 255, 255)],
            'orange': [(10, 50, 50), (25, 255, 255)],
            'purple': [(130, 50, 50), (170, 255, 255)]
        }
        
        self.min_area = 500
        
    def create_mask(self, hsv_image, color_name):
        if color_name not in self.color_ranges:
            return None
            
        color_range = self.color_ranges[color_name]
        
        if color_name == 'red':
            lower1, upper1, lower2, upper2 = color_range
            mask1 = cv2.inRange(hsv_image, np.array(lower1), np.array(upper1))
            mask2 = cv2.inRange(hsv_image, np.array(lower2), np.array(upper2))
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            lower, upper = color_range
            mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
            
        return mask
    
    def find_contours(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.min_area]
        return filtered_contours
    
    def draw_bounding_boxes(self, image, contours, color_name, color_bgr):
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            
            cv2.rectangle(image, (x, y), (x + w, y + h), color_bgr, 2)
            
            label = f"{color_name}: {w}x{h}"
            cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)
            
        return image
    
    def segment_colors(self, image):
        segment_start = time.perf_counter()
        print(f"  [COLOR] Starting color segmentation (image shape: {image.shape})")
        
        if image is None:
            return None
            
        hsv_start = time.perf_counter()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv_time = time.perf_counter() - hsv_start
        print(f"  [COLOR] HSV conversion took: {hsv_time*1000:.3f}ms")
        
        result_image = image.copy()
        
        color_bgr_map = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'yellow': (0, 255, 255),
            'orange': (0, 165, 255),
            'purple': (128, 0, 128)
        }
        
        detected_objects = []
        
        processing_start = time.perf_counter()
        total_objects = 0
        for color_name in self.color_ranges.keys():
            mask = self.create_mask(hsv, color_name)
            if mask is None:
                continue
                
            contours = self.find_contours(mask)
            
            if contours:
                total_objects += len(contours)
                color_bgr = color_bgr_map.get(color_name, (255, 255, 255))
                result_image = self.draw_bounding_boxes(result_image, contours, color_name, color_bgr)
                
                for contour in contours:
                    x, y, w, h = cv2.boundingRect(contour)
                    detected_objects.append({
                        'color': color_name,
                        'bbox': (x, y, w, h),
                        'area': cv2.contourArea(contour)
                    })
        
        processing_time = time.perf_counter() - processing_start
        print(f"  [COLOR] Color processing took: {processing_time*1000:.3f}ms (found {total_objects} objects)")
        
        total_time = time.perf_counter() - segment_start
        print(f"  [COLOR] Total segmentation took: {total_time*1000:.3f}ms")
        
        return result_image, detected_objects

def process_color_segmentation(image):
    segmenter = ColorSegmenter()
    result = segmenter.segment_colors(image)
    
    if result is None:
        return image, []
        
    result_image, detected_objects = result
    
    if detected_objects:
        print(f"Detected {len(detected_objects)} colored objects:")
        for obj in detected_objects:
            print(f"  {obj['color']}: bbox={obj['bbox']}, area={obj['area']:.1f}")
    
    return result_image, detected_objects