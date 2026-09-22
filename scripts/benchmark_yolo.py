"""Offline local-model benchmark; it never starts a simulator or a vehicle process."""
import argparse
import statistics
import time
from pathlib import Path


def measure(model, image, runs, warmup, options):
    for _ in range(warmup):
        model.predict(image, **options)
    durations = []
    for _ in range(runs):
        started = time.perf_counter()
        model.predict(image, **options)
        durations.append((time.perf_counter()-started)*1000.0)
    average = statistics.mean(durations)
    return average, 1000.0/average if average else 0.0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="output/annotated_frame.png")
    parser.add_argument("--nano", default="models/yolo26n.pt")
    parser.add_argument("--large", default="models/yolo26l.pt")
    parser.add_argument("--engine", default="models/yolo26l.engine")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--runs", type=int, default=30)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--data", help="Optional labelled dataset YAML for mAP validation")
    args = parser.parse_args(argv)
    if args.runs <= 0 or args.warmup < 0:
        parser.error("runs must be positive and warmup non-negative")
    import cv2
    from ultralytics import YOLO
    image = cv2.imread(args.source)
    if image is None:
        parser.error(f"Cannot read source image: {args.source}")
    print("| model | format | mAP50-95 | mean inference | FPS |")
    print("|---|---|---:|---:|---:|")
    for label, model_path in (("nano", args.nano), ("large", args.large), ("large FP16", args.engine)):
        path = Path(model_path)
        if not path.is_file():
            print(f"| {label} | unavailable | not measured | not measured | not measured |")
            continue
        model = YOLO(str(path))
        elapsed_ms, fps = measure(model, image, args.runs, args.warmup,
                                  {"device": args.device, "verbose": False})
        map_value = "not measured"
        if args.data:
            map_value = f"{model.val(data=args.data, device=args.device, verbose=False).box.map:.4f}"
        print(f"| {label} | {path.suffix.lstrip('.') or 'checkpoint'} | {map_value} | {elapsed_ms:.2f} ms | {fps:.2f} |")


if __name__ == "__main__":
    main()
