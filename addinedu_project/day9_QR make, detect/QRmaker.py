import qrcode
from PIL import Image
from pathlib import Path
from datetime import datetime

# 로고 삽입 함수
def add_logo(qr_img, logo_path):
    logo = Image.open(logo_path)
    qr_width, qr_height = qr_img.size

    factor = 4
    size_w = int(qr_width / factor)
    size_h = int(qr_height / factor)
    logo = logo.resize((size_w, size_h), Image.LANCZOS)

    pos = ((qr_width - size_w) // 2, (qr_height - size_h) // 2)
    qr_img.paste(logo, pos, mask=logo.convert('RGBA'))

    return qr_img

# 날짜 기반 파일명 생성 함수
def generate_unique_filename(base_name="qrcode", ext=".png", save_dir=Path(".")):
    today = datetime.now().strftime("%Y-%m-%d")
    count = 1

    while True:
        filename = f"{base_name}_{today}_{count}{ext}"
        full_path = save_dir / filename
        if not full_path.exists():
            return full_path
        count += 1

# 저장할 폴더 설정 (예: ~/Pictures/qrcode)
save_dir = Path("/home/jetcobot/main/data/Custom_QR")
save_dir.mkdir(parents=True, exist_ok=True)  # 폴더 없으면 생성

# 데이터 내용 (QR에 담길 링크나 텍스트)
data = "https://github.com/roastB/ros2study"

# QR 코드 생성
qr = qrcode.QRCode(
    version=1,
    error_correction=qrcode.constants.ERROR_CORRECT_H,
    box_size=10,
    border=4,
)
qr.add_data(data)
qr.make(fit=True)

img = qr.make_image(fill_color="black", back_color="white").convert('RGB')

# 로고 삽입
logo_path = "/home/addineud/Downloads/logo.png"

if Path(logo_path).is_file():
    img = add_logo(img, logo_path)
    print("✅ 로고 삽입 완료")
else:
    print("🐱 로고 없음: 기본 QR 코드 생성")

# 고유 파일명 생성 및 저장
file_path = generate_unique_filename(base_name="QR", save_dir=save_dir)
img.save(file_path)
print(f"📁 저장 완료: {file_path}")
