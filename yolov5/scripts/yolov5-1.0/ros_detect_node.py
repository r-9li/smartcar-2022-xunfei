#!/usr/bin/env python3
# encoding=utf8
import argparse

import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String
import os

import torch.backends.cudnn as cudnn

from utils import google_utils
from utils.datasets import *
from utils.utils import *

################################全局维护变量######################################
debug_save_image = True
image_temp_path = "/home/ucar/ucar_ws/src/ucar_nav/image_temp"


################################################################################


def detect(file_path):
    # Set Dataloader
    global debug_save_image
    save_img = debug_save_image
    vid_path, vid_writer = None, None
    source = file_path
    output_list = []

    dataset = LoadImages(source, img_size=imgsz)

    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]

    # Run inference
    t0 = time.time()
    img = torch.zeros((1, 3, imgsz, imgsz), device=device)  # init img
    _ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once
    for path, img, im0s, vid_cap in dataset:
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = torch_utils.time_synchronized()
        pred = model(img, augment=opt.augment)[0]

        # Apply NMS
        pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
        t2 = torch_utils.time_synchronized()

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            p, s, im0 = path, '', im0s

            save_path = str(Path(out) / Path(p).name)
            s += '%gx%g ' % img.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  #  normalization gain whwh
            if det is not None and len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += '%g %ss, ' % (n, names[int(c)])  # add to string

                # Write results
                for *xyxy, conf, cls in det:
                    output_list.append(int(cls))
                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        with open(save_path[:save_path.rfind('.')] + '.txt', 'a') as file:
                            file.write(('%g ' * 5 + '\n') % (cls, *xywh))  # label format

                    if save_img or view_img:  # Add bbox to image
                        label = '%s %.2f' % (names[int(cls)], conf)
                        plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=3)

            # Print time (inference + NMS)
            print('%sDone. (%.3fs)' % (s, t2 - t1))

            # Stream results
            if view_img:
                cv2.imshow(p, im0)
                if cv2.waitKey(1) == ord('q'):  # q to quit
                    raise StopIteration

            # Save results (image with detections)
            if save_img:
                if dataset.mode == 'images':
                    cv2.imwrite(save_path, im0)
                else:
                    if vid_path != save_path:  # new video
                        vid_path = save_path
                        if isinstance(vid_writer, cv2.VideoWriter):
                            vid_writer.release()  # release previous video writer

                        fps = vid_cap.get(cv2.CAP_PROP_FPS)
                        w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*opt.fourcc), fps, (w, h))
                    vid_writer.write(im0)

    if save_txt or save_img:
        print('Results saved to %s' % os.getcwd() + os.sep + out)
        if platform == 'darwin':  # MacOS
            os.system('open ' + save_path)

    print('Done. (%.3fs)' % (time.time() - t0))
    return output_list


def callback(data):
    global image_temp_path
    if data.data == 1:
        room_temp_list_1 = []
        room_temp_list_2 = []
        room_temp_list_3 = []
        pub_string = ""
        with torch.no_grad():
            filename_list = os.listdir(image_temp_path)
            for filename in filename_list:
                temp_type_list = detect(image_temp_path + '/' + filename)
                if int(filename[0]) == 1:
                    room_temp_list_1 += temp_type_list
                elif int(filename[0]) == 2:
                    room_temp_list_2 += temp_type_list
                elif int(filename[0]) == 3:
                    room_temp_list_3 += temp_type_list
            room_temp_list_1 = list(set(room_temp_list_1))
            room_temp_list_2 = list(set(room_temp_list_2))
            room_temp_list_3 = list(set(room_temp_list_3))
            for room_type in room_temp_list_1:
                pub_string += str(room_type)
            pub_string += ','
            for room_type in room_temp_list_2:
                pub_string += str(room_type)
            pub_string += ','
            for room_type in room_temp_list_3:
                pub_string += str(room_type)
            pub_string += ','
            pub_string_temp = String()
            pub_string_temp.data = pub_string
            pub_detect.publish(pub_string_temp)


if __name__ == '__main__':
    rospy.init_node('yolo_detect', anonymous=True)
    rospy.Subscriber("/yolo_detect", Int8, callback)
    pub_detect = rospy.Publisher("/detect", String, queue_size=10)
    # ###########################################################初始化#######################################################
    time_temp = rospy.Time.now()
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default='/home/ucar/ucar_ws/src/yolov5/scripts/yolov5-1.0/best.pt', help='model.pt path')
    parser.add_argument('--source', type=str, default='inference/images', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--output', type=str, default='/home/ucar/ucar_ws/src/yolov5/scripts/yolov5-1.0/inference/output', help='output folder')  # output folder
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.4, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.5, help='IOU threshold for NMS')
    parser.add_argument('--fourcc', type=str, default='mp4v', help='output video codec (verify ffmpeg support)')
    parser.add_argument('--device', default='0', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    opt = parser.parse_args()
    opt.img_size = check_img_size(opt.img_size)
    print(opt)
    out, source, weights, view_img, save_txt, imgsz = \
        opt.output, opt.source, opt.weights, opt.view_img, opt.save_txt, opt.img_size

    # Initialize
    device = torch_utils.select_device(opt.device)
    if os.path.exists(out):
        shutil.rmtree(out)  # delete output folder
    os.makedirs(out)  # make new output folder
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    google_utils.attempt_download(weights)
    model = torch.load(weights, map_location=device)['model'].float()  # load to FP32
    # torch.save(torch.load(weights, map_location=device), weights)  # update model if SourceChangeWarning
    # model.fuse()
    model.to(device).eval()
    if half:
        model.half()  # to FP16
    # #######################################################################################################################
    print(rospy.Time.to_time(rospy.Time.now() - time_temp))  # 加载时间
    rospy.spin()
