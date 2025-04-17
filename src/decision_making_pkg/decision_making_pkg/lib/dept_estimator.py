import os
import cv2
import torch
import torchvision

from .midas.midas_net_custom import MidasNet_small
from .midas.transforms import Resize, NormalizeImage, PrepareForNet

def dept_estimator(img):
    # GPU 사용여부 확인
    device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")

    # MIDAS 모델 호출
    name = os.path.dirname(__file__) + "/" + "midas_v21_small_256.pt"

    param = torch.load(name, map_location=device)
    model = MidasNet_small()

    # 파라미터 반영
    model.load_state_dict(param)

    # 사진 변환 함수 선언
    transform = torchvision.transforms.Compose(
            [
            lambda img: {"image": img / 255.0},
            Resize(
                256,
                256,
                resize_target=None,
                keep_aspect_ratio=True,
                ensure_multiple_of=32,
                resize_method="upper_bound",
                image_interpolation_method=cv2.INTER_CUBIC,
            ),
            NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            PrepareForNet(),
            lambda sample: torch.from_numpy(sample["image"]).unsqueeze(0),
        ]
    )

    input_batch = transform(img)

    # 이미지 호출
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # 행렬 연산 처리
    with torch.no_grad():
        prediction = model(input_batch)

        prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=img.shape[:2],
            mode="bicubic",
            align_corners=False,
        ).squeeze()

    return prediction.cpu().numpy()