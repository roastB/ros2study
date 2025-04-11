import cv2 as cv
import numpy as np
from pyzbar import pyzbar
from PIL import Image, ImageDraw, ImageFont

def decodeDisplay(image, font_path):
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    barcodes = pyzbar.decode(gray)

    for barcode in barcodes:
        (x, y, w, h) = barcode.rect
        cv.rectangle(image, (x, y), (x + w, y + h), (225, 0, 0), 2)

        try:
            encoding = 'utf-8'
            barcodeData = barcode.data.decode(encoding)
        except UnicodeDecodeError:
            print("[WARN] Unicode decoding failed. Using raw bytes.")
            barcodeData = str(barcode.data)

        barcodeType = barcode.type

        try:
            pil_img = Image.fromarray(cv.cvtColor(image, cv.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(pil_img)
            font = ImageFont.load_default()  # 기본 내장 폰트
            draw.text((x, y - 25), barcodeData, fill=(255, 0, 0), font=font)
            image = cv.cvtColor(np.array(pil_img), cv.COLOR_RGB2BGR)
        except Exception as e:
            print(f"[ERROR] Drawing text failed: {e}")

        print(f"[INFO] Found {barcodeType} barcode: {barcodeData}")

    return image

# ✅ 여기부터 메인 루프
if __name__ == "__main__":
    font_path = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"  # 시스템에 맞게 조정 필요
    cap = cv.VideoCapture(0)

    if not cap.isOpened():
        print("❌ 카메라를 열 수 없습니다.")
        exit()

    print("📷 카메라 실행 중... QR 코드를 보여주세요. (종료: q 키)")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 프레임을 가져올 수 없습니다.")
            break

        frame = decodeDisplay(frame, font_path)
        cv.imshow("QR Detection", frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            print("👋 종료합니다.")
            break

    cap.release()
    cv.destroyAllWindows()

