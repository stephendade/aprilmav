'''
Simple GUI for displaying AprilAMV status
'''
import os
import cv2
import numpy
import matplotlib.pyplot as plt

from modules.common import getFontSize, labelTags


class GUI:
    '''
    A simple GUI for displaying camera image, tags and vehicle position and rotation
    '''
    def __init__(self):
        self.fig = None
        self.axRot = None
        self.axPos = None
        self.coordsX = []
        self.coordsY = []
        self.coordsZ = []
        self.rotX = []
        self.rotY = []
        self.rotZ = []

        self.text_height = None
        self.font_scale = 1
        self.thickness = 4

        print("plotting")

        # Add env var for QT_QPA_PLATFORM=xcb
        os.environ['QT_QPA_PLATFORM'] = 'xcb'

        # Create a figure containing a single axes.
        self.fig, (self.axRot, self.axPos) = plt.subplots(2)
        self.axRot.set_xlim(0, 10)
        self.axRot.set_ylim(-180, 180)
        self.axPos.set_xlim(0, 10)
        self.axPos.set_ylim(-4, 4)
        self.axRot.set(xlabel='', ylabel='Rotation (deg)', title='Vehicle Rotation')
        self.axPos.set(xlabel='', ylabel='Distance (m)', title='Vehicle Position')
        self.axRot.set_yticks(numpy.arange(-180, 181, 45))
        # add gridlines
        self.axRot.grid()
        self.axPos.grid()
        # Plot some data on the axes.
        self.lineX, = self.axPos.plot([], [])
        self.lineY, = self.axPos.plot([], [])
        self.lineZ, = self.axPos.plot([], [])
        self.line_rotX, = self.axRot.plot([], [])
        self.line_rotY, = self.axRot.plot([], [])
        self.line_rotZ, = self.axRot.plot([], [])

        plt.tight_layout()
        self.fig.canvas.manager.set_window_title('AprilMAV Status')

        dpi = self.fig.dpi
        self.fig.set_size_inches(600 / dpi, 800 / dpi)

        plt.show(block=False)

        # OpenCV window for images
        cv2.namedWindow("Rectified image with tags", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Rectified image with tags", 700, 700)

    def update(self, posR, rotD):
        '''
        Update the plot with new data
        '''
        self.coordsX.append(posR[0])
        self.coordsY.append(posR[1])
        self.coordsZ.append(posR[2])
        self.rotX.append(rotD[0])
        self.rotY.append(rotD[1])
        self.rotZ.append(rotD[2])
        xAxisData = numpy.arange(0, len(self.coordsY), 1)
        self.lineX.set_xdata(xAxisData)
        self.lineX.set_ydata(self.coordsX)
        self.lineY.set_xdata(xAxisData)
        self.lineY.set_ydata(self.coordsY)
        self.lineZ.set_xdata(xAxisData)
        self.lineZ.set_ydata(self.coordsZ)
        self.line_rotX.set_xdata(xAxisData)
        self.line_rotX.set_ydata(self.rotX)
        self.line_rotY.set_xdata(xAxisData)
        self.line_rotY.set_ydata(self.rotY)
        self.line_rotZ.set_xdata(xAxisData)
        self.line_rotZ.set_ydata(self.rotZ)
        # update the y axis range to fit (for posn)
        self.axPos.set_ylim(min(self.coordsX + self.coordsY + self.coordsZ)-1,
                            max(self.coordsX + self.coordsY + self.coordsZ)+1)
        self.axPos.set_xlim(0, max(len(self.coordsY), 50))
        self.axRot.set_xlim(0, max(len(self.rotY), 50))

        # add legend
        self.axRot.legend(['Roll', 'Pitch', 'Yaw'])
        self.axPos.legend(['Fwd', 'Right', 'Down'])
        plt.draw()
        self.fig.canvas.flush_events()

    def updateImage(self, CAMERAS):
        '''
        Update the image window with a new image
        '''
        imageColour = None
        for CAMERA in CAMERAS:
            imageCam = cv2.cvtColor(CAMERA.imageBW, cv2.COLOR_GRAY2BGR)

            if not self.text_height:
                self.font_scale, self.thickness, self.text_height = getFontSize(imageCam)

            if CAMERA.tags:
                imageCam = labelTags(imageCam, CAMERA.tags, self.thickness,
                                     self.font_scale)

            cv2.putText(imageCam, CAMERA.camName, (10, self.text_height + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, (255, 0, 0), self.thickness)

            # put a border around the image
            imageCam = cv2.copyMakeBorder(
                imageCam, 15, 15, 15, 15, cv2.BORDER_CONSTANT, value=(255, 255, 255))
            # append to final image
            if imageColour is None:
                imageColour = imageCam
            else:
                imageColour = cv2.hconcat([imageColour, imageCam])
        cv2.imshow("Rectified image with tags", imageColour)
        cv2.waitKey(1) & 0xFF == ord('0')

    def on_end(self):
        print("Waiting for plot window to be closed")
        plt.show()
        cv2.waitKey(1) & 0xFF == ord('0')
        cv2.destroyAllWindows()
