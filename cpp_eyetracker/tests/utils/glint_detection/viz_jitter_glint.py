import cv2
import dlib
import os
import glob
import imageio
from tqdm import tqdm
import yaml

CONFIG_PATH = "D:/hitsz/projects/new_dataset/remote-eye-system/cpp_eyetracker/cfg/default.yaml"

def create_eye_videos_fixed_pos(input_folder, model_path, fps=30, padding=30):
    detector = dlib.get_frontal_face_detector()
    predictor = dlib.shape_predictor(model_path)

    extensions = ('*.jpg', '*.jpeg', '*.png')
    image_files = []
    for ext in extensions:
        image_files.extend(glob.glob(os.path.join(input_folder, ext)))
    image_files.sort()

    if not image_files:
        print("Error: 未找到图片。")
        return

    first_frame = cv2.imread(image_files[0])
    gray = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)
    faces = detector(gray)

    if not faces:
        print("Error: 未检测到人脸。")
        return

    shape = predictor(gray, faces[0])

    def get_fixed_box_even(indices, pad):
        coords = [(shape.part(i).x, shape.part(i).y) for i in indices]
        x1 = min(c[0] for c in coords) - pad
        y1 = min(c[1] for c in coords) - pad
        x2 = max(c[0] for c in coords) + pad
        y2 = max(c[1] for c in coords) + pad
        
        # --- 核心修复：确保宽高为偶数 ---
        w = x2 - x1
        h = y2 - y1
        if w % 2 != 0: x2 += 1  # 如果宽是奇数，向右扩展1像素
        if h % 2 != 0: y2 += 1  # 如果高是奇数，向下扩展1像素
        
        return int(x1), int(y1), int(x2), int(y2)

    # 左右眼索引对调
    idx_left_eye = range(42, 48) 
    idx_right_eye = range(36, 42)

    l_x1, l_y1, l_x2, l_y2 = get_fixed_box_even(idx_left_eye, padding)
    r_x1, r_y1, r_x2, r_y2 = get_fixed_box_even(idx_right_eye, padding)

    # 打印一下尺寸确保是偶数
    print(f"左眼尺寸: {l_x2-l_x1}x{l_y2-l_y1}, 右眼尺寸: {r_x2-r_x1}x{r_y2-r_y1}")

    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
    use_glint_sr = cfg['test_glint']['use_glint_sr']
    left_video_name = "left_eye.mp4"
    right_video_name = "right_eye.mp4"

    if use_glint_sr:
        left_video_name = "left_eye_sr.mp4"
        right_video_name = "right_eye_sr.mp4"

    path_l = os.path.join(input_folder, left_video_name)
    path_r = os.path.join(input_folder, right_video_name)

    # 高质量参数
    writer_args = {
        'fps': fps,
        'codec': 'libx264',
        'pixelformat': 'yuv420p',
        'macro_block_size': 1,
        'ffmpeg_params': ['-crf', '12']  # CRF 12 为极高质量
    }

    writer_l = imageio.get_writer(path_l, format='FFMPEG', mode='I', **writer_args)
    writer_r = imageio.get_writer(path_r, format='FFMPEG', mode='I', **writer_args)

    for img_path in tqdm(image_files, desc="Processing"):
        frame = cv2.imread(img_path)
        if frame is None: continue

        # 裁剪
        crop_l = frame[max(0, l_y1):l_y2, max(0, l_x1):l_x2]
        crop_r = frame[max(0, r_y1):r_y2, max(0, r_x1):r_x2]

        # 转换并写入
        writer_l.append_data(cv2.cvtColor(crop_l, cv2.COLOR_BGR2RGB))
        writer_r.append_data(cv2.cvtColor(crop_r, cv2.COLOR_BGR2RGB))

    writer_l.close()
    writer_r.close()
    print("\n成功生成！尺寸已自动修正为偶数。")

if __name__ == "__main__":
    folder = r"D:\hitsz\projects\new_dataset\eyetracker_test\sby\record_20260408_152345\cam_0\debug_img"
    model_dat = r"D:\downloads\chrome_downloads\apps\shape_predictor_68_face_landmarks.dat"
    create_eye_videos_fixed_pos(folder + "/6_jitter_glint", model_dat, fps=30)