"""Export a local detector checkpoint to a TensorRT FP16 engine; no simulation is started."""
import argparse
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="models/yolo26s.pt")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    args = parser.parse_args(argv)
    if args.imgsz <= 0:
        parser.error("imgsz must be positive")
    model_path = Path(args.model)
    if not model_path.is_file():
        parser.error(f"Missing local model: {model_path}")
    from ultralytics import YOLO
    output = YOLO(str(model_path)).export(format="engine", half=True, imgsz=args.imgsz, device=args.device)
    print(f"Saved: {output}")


if __name__ == "__main__":
    main()
