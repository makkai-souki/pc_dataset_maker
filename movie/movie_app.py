import os
import cv2


class Movie():
    def __init__(self):
        pass

    def frame(self, file, directry, freq=1, magnification=1, ext='.png'):
        self.movie_app = MovieApp()
        self.movie_app.set_filename(file)
        self.movie_app.set_directry_name(directry)
        self.movie_app.set_freq(freq)
        self.movie_app.set_magnification(magnification)
        self.movie_app.set_extension(ext)
        self.movie_app.make_directry()
        self.movie_app.capture_video()
        self.movie_app.frame_loop()


class MovieApp():
    def set_filename(self, file):
        self.file = './video/{}'.format(file)

    def set_directry_name(self, directry):
        self.directry = './frames/{}'.format(directry)

    def set_freq(self, freq):
        self.freq = int(freq)

    def set_magnification(self, magnification):
        self.magnification = float(magnification)

    def set_extension(self, ext):
        self.ext = ext

    def make_directry(self):
        os.makedirs(self.directry, exist_ok=True)

    def capture_video(self):
        self.cap = cv2.VideoCapture(self.file)

    def frame_loop(self):
        frame_number = int(0)
        while True:
            is_captured, c_frame = self.cap.read()
            if is_captured:
                if frame_number % self.freq == 0:
                    new_frame = self.resize_frame(c_frame)
                    cv2.imwrite(self.directry + '/' + str((100000 + frame_number) * 100000) + self.ext, new_frame)
            else:
                break
            frame_number += 1
            if frame_number % 100 == 0:
                print(frame_number)
                self.print_size(new_frame)

    def resize_frame(self, frame):
        height = frame.shape[0]
        width = frame.shape[1]
        return cv2.resize(frame, (int(width * self.magnification), int(height * self.magnification)))

    def print_size(self, frame):
        print('width:{}, height: {}'.format(frame.shape[1], frame.shape[0]))