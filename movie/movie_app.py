import os
import cv2


class MovieApp():
    def __init__(self):
        pass

    def frame(self, file, directry, fps=30):
        file_name = './video/{}'.format(file)
        directry = './frames/{}'.format(directry)
        os.makedirs(directry)
        cap = cv2.VideoCapture(file_name)
        frame_number = int(0)
        end_frag, c_frame = cap.read()
        while end_frag:
            cv2.imwrite(directry + '/' + str((10000 + frame_number)
                                             * 10000000) + '.png', c_frame)
            frame_number += 1
            print(frame_number)
            end_flag, c_frame = cap.read()
