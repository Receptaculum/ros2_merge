import os
import cv2
import torch
import torchvision

import matplotlib.pyplot as plt

from .midas.midas_net_custom import MidasNet_small
from .midas.transforms import Resize, NormalizeImage, PrepareForNet


def depth_estimator(img):

    # 속도 최적화를 위한 설정
    torch.set_flush_denormal(True)
    # torch.multiprocessing.set_start_method('spawn')

    # GPU 사용여부 확인
    if torch.cuda.is_available():
        device = torch.device("cuda") 

    else:
        torch.set_num_threads(4)
        device = torch.device("cpu")

    # MIDAS 모델 호출
    name = os.path.dirname(__file__) + "/" + "midas_v21_small_256_16bit.pt"
    param = torch.load(name, map_location=device)
    model = MidasNet_small()

    # 추론 모드 설정
    model.eval()

    # 모델에 파라미터 대입
    model.load_state_dict(param)

    # 이미지 변환 Object 선언
    transform = torchvision.transforms.Compose(
            [
            lambda img: {"image": img / 255.0},
            #Resize(
            #    256,
            #    256,
            #    resize_target=None,
            #    keep_aspect_ratio=True,
            #    ensure_multiple_of=32,
            #    resize_method="upper_bound",
            #    image_interpolation_method=cv2.INTER_CUBIC,
            #),
            NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            PrepareForNet(),
            lambda sample: torch.from_numpy(sample["image"]).unsqueeze(0),
        ]
    )

    # 이미지 가공
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (256, 256))
    input_batch = transform(img)

    # 연산 처리
    with torch.no_grad():
        prediction = model(input_batch).squeeze().cpu().numpy()

    return cv2.resize(prediction, (640, 480))
