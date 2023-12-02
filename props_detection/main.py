import cv2
import os
import numpy as np
from concurrent.futures import ProcessPoolExecutor, as_completed


def image_comparison(image_path, frame):
    x = cv2.imread(image_path)
    return np.sum((x - frame)**2)


def main():
    cap = cv2.VideoCapture(1)
    path = "./props_detection/image/"

    with ProcessPoolExecutor() as executor:
        while True:
            _, frame = cap.read()
            y = []
            D = []

            future_to_filename = {executor.submit(image_comparison, os.path.join(
                path, fname), frame): fname for fname in os.listdir(path) if '.png' in fname}

            for future in as_completed(future_to_filename):
                fname = future_to_filename[future]
                try:
                    result = future.result()
                    D.append(result)
                    y.append(fname.split('_')[0])
                except Exception as e:
                    print(f"Error processing {fname}: {e}")

            if len(D) > 0:
                ans = y[D.index(min(D))]

                if ans in ["propblue", "propred"]:
                    cv2.putText(frame, ans, (10, 20),
                                cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255))
                    print("The object is:", ans)
                else:
                    ans = "none"
                    print(ans)

            cv2.imshow('frame', frame)
            if cv2.waitKey(1) == 27:
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
