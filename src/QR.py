import cv2
import numpy as np
from pyzbar import pyzbar
import time
import os
import shutil
from datetime import datetime

# 全局变量
img = None
last_qr_result = None
last_qr_display = None
last_scan_time = 0
SCAN_INTERVAL = 2  # 每2秒扫描一次

def order_points(pts):
    """对四个角点进行排序：左上、右上、右下、左下"""
    rect = np.zeros((4, 2), dtype=np.float32)
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]  # 左上
    rect[2] = pts[np.argmax(s)]  # 右下
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # 右上
    rect[3] = pts[np.argmax(diff)]  # 左下
    return rect

def preprocess_image(gray):
    """多种预处理方法提高识别率"""
    processed_images = []
    
    # 1. 原始灰度图
    processed_images.append(("原始", gray))
    
    # 2. CLAHE 增强对比度
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    processed_images.append(("CLAHE", enhanced))
    
    # 3. 自适应二值化
    adaptive = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
        cv2.THRESH_BINARY, 11, 2
    )
    processed_images.append(("自适应", adaptive))
    
    # 4. OTSU 二值化
    _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    processed_images.append(("OTSU", otsu))
    
    # 5. 锐化
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    sharpened = cv2.addWeighted(gray, 1.5, blurred, -0.5, 0)
    processed_images.append(("锐化", sharpened))
    
    return processed_images

def detect_and_enhance_qrcode(frame):
    """检测、矫正、增强二维码"""
    start_time = time.time()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    processed_images = preprocess_image(gray)
    
    # 在每种预处理图像上尝试识别
    for method_name, processed in processed_images:
        qr_codes = pyzbar.decode(processed)
        
        if qr_codes:
            for qr in qr_codes:
                points = qr.polygon
                
                if len(points) == 4:
                    pts = np.array([[p.x, p.y] for p in points], dtype=np.float32)
                    pts = order_points(pts)
                    
                    # 计算尺寸
                    width = int(max(
                        np.linalg.norm(pts[0] - pts[1]),
                        np.linalg.norm(pts[2] - pts[3])
                    ))
                    height = int(max(
                        np.linalg.norm(pts[1] - pts[2]),
                        np.linalg.norm(pts[3] - pts[0])
                    ))
                    
                    size = max(500, max(width, height))
                    
                    # 透视变换
                    dst_pts = np.array([
                        [0, 0],
                        [size, 0],
                        [size, size],
                        [0, size]
                    ], dtype=np.float32)
                    
                    matrix = cv2.getPerspectiveTransform(pts, dst_pts)
                    warped = cv2.warpPerspective(gray, matrix, (size, size))
                    
                    # 增强对比度
                    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
                    enhanced = clahe.apply(warped)
                    
                    # 二值化
                    _, binary = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                    
                    qr_data = qr.data.decode('utf-8', errors='ignore')
                    elapsed = time.time() - start_time
                    return binary, qr_data, True, method_name, elapsed
    
    # 使用 OpenCV QRCodeDetector 作为备用
    detector = cv2.QRCodeDetector()
    for method_name, processed in processed_images[:2]:  # 只在前2种方法上尝试
        try:
            data, bbox, _ = detector.detectAndDecode(processed)
            
            if bbox is not None and data:
                bbox = bbox[0].astype(np.float32)
                bbox = order_points(bbox)
                
                width = int(max(
                    np.linalg.norm(bbox[0] - bbox[1]),
                    np.linalg.norm(bbox[2] - bbox[3])
                ))
                height = int(max(
                    np.linalg.norm(bbox[1] - bbox[2]),
                    np.linalg.norm(bbox[3] - bbox[0])
                ))
                
                size = max(500, max(width, height))
                
                dst_pts = np.array([
                    [0, 0],
                    [size, 0],
                    [size, size],
                    [0, size]
                ], dtype=np.float32)
                
                matrix = cv2.getPerspectiveTransform(bbox, dst_pts)
                warped = cv2.warpPerspective(gray, matrix, (size, size))
                
                clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
                enhanced = clahe.apply(warped)
                _, binary = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                
                elapsed = time.time() - start_time
                return binary, data, True, f"OpenCV-{method_name}", elapsed
        except Exception:
            continue
    
    elapsed = time.time() - start_time
    return None, None, False, None, elapsed

def setup_output_folder():
    """设置输出文件夹，如果存在则清空"""
    base_folder = "QR_Code"
    qr_folder = os.path.join(base_folder, "qr_images")
    photo_folder = os.path.join(base_folder, "photos")
    
    if os.path.exists(base_folder):
        shutil.rmtree(base_folder)
        print(f"已清空文件夹: {base_folder}")
    
    os.makedirs(qr_folder)
    os.makedirs(photo_folder)
    print(f"已创建文件夹: {qr_folder}")
    print(f"已创建文件夹: {photo_folder}")
    
    return qr_folder, photo_folder

def main():
    global img, last_qr_result, last_qr_display, last_scan_time
    
    qr_folder, photo_folder = setup_output_folder()
    
    # 创建窗口
    cv2.namedWindow('Camera Feed')
    cv2.namedWindow('QR Code Result')
    
    # 初始化摄像头
    cm0 = cv2.VideoCapture(0)
    cm0.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cm0.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cm0.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cm0.set(cv2.CAP_PROP_FPS, 30)
    
    if not cm0.isOpened():
        print("无法打开摄像头")
        return
    
    print("="*60)
    print("树莓派 4 二维码扫描器")
    print(f"扫描间隔: {SCAN_INTERVAL} 秒")
    print("每次扫描会同时保存原始照片和二维码")
    print("按 's' 键手动扫描，按 'q' 键退出")
    print("="*60 + "\n")
    
    scan_count = 0
    total_scan_time = 0
    
    # 初始化结果窗口
    blank_result = np.ones((400, 400), dtype=np.uint8) * 200
    cv2.putText(blank_result, "Waiting for QR Code...", (50, 200),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)
    cv2.imshow('QR Code Result', blank_result)
    last_qr_display = blank_result
    
    while True:
        ret, img = cm0.read()
        if not ret:
            continue
        
        current_time = time.time()
        display = img.copy()
        
        # 显示状态信息
        time_since_last = current_time - last_scan_time
        next_scan_in = max(0, SCAN_INTERVAL - time_since_last)
        
        cv2.putText(display, f"Next: {next_scan_in:.1f}s", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display, f"Count: {scan_count}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        if scan_count > 0:
            avg_time = total_scan_time / scan_count
            cv2.putText(display, f"Avg: {avg_time:.2f}s", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        if last_qr_result:
            display_text = last_qr_result[:15] + "..." if len(last_qr_result) > 15 else last_qr_result
            cv2.putText(display, f"Last: {display_text}", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        cv2.imshow('Camera Feed', display)
        
        # 检查按键
        should_scan = False
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('s'):
            should_scan = True
            print("手动扫描触发")
        elif key == ord('q'):
            break
        elif current_time - last_scan_time >= SCAN_INTERVAL:
            should_scan = True
        
        if should_scan:
            last_scan_time = current_time
            print(f"\n[扫描 #{scan_count + 1}]")
            
            # 保存原始照片
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            photo_filename = f"photo_{timestamp}.jpg"
            photo_filepath = os.path.join(photo_folder, photo_filename)
            cv2.imwrite(photo_filepath, img)
            print(f"📷 照片已保存: {photo_filename}")
            
            # 执行二维码检测
            enhanced_qr, qr_data, success, method, elapsed = detect_and_enhance_qrcode(img)
            total_scan_time += elapsed
            
            if success and enhanced_qr is not None:
                scan_count += 1
                qr_filename = f"qr_{timestamp}_{scan_count}.png"
                qr_filepath = os.path.join(qr_folder, qr_filename)
                
                cv2.imwrite(qr_filepath, enhanced_qr)
                
                # 显示结果
                h, w = enhanced_qr.shape
                if h != 400 or w != 400:
                    result_display = cv2.resize(enhanced_qr, (400, 400))
                else:
                    result_display = enhanced_qr
                
                cv2.imshow('QR Code Result', result_display)
                last_qr_display = result_display
                
                print(f"✓ 识别成功 ({elapsed:.2f}秒, 方法: {method})")
                print(f"  内容: {qr_data}")
                print(f"  二维码: {qr_filename}")
                
                last_qr_result = qr_data
            else:
                print(f"✗ 未检测到 ({elapsed:.2f}秒) → 保持上次结果")
                if last_qr_display is not None:
                    cv2.imshow('QR Code Result', last_qr_display)
    
    # 清理
    cm0.release()
    cv2.destroyAllWindows()
    
    # 统计保存的文件数量
    photo_count = len([f for f in os.listdir(photo_folder) if f.endswith('.jpg')])
    
    print("\n" + "="*60)
    print(f"程序结束")
    print(f"识别到二维码: {scan_count} 个")
    print(f"保存照片: {photo_count} 张")
    if scan_count > 0:
        print(f"平均用时: {total_scan_time/scan_count:.2f} 秒")
    print(f"二维码保存在: {qr_folder}")
    print(f"照片保存在: {photo_folder}")
    print("="*60)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
        cv2.destroyAllWindows()
    except Exception as e:
        print(f"发生错误: {e}")
        import traceback
        traceback.print_exc()
        cv2.destroyAllWindows()
