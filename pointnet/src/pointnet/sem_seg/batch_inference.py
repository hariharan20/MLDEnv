#!/usr/bin/env python

import rospy
import argparse
import os
import sys
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(BASE_DIR)
from model import *
import indoor3d_util
from sensor_msgs.msg import PointCloud2
import pcl
import ros_numpy
from geometry_msgs.msg import Point
import numpy as np
count = 0
parser = argparse.ArgumentParser()
parser.add_argument('--gpu', type=int, default=5, help='GPU to use [default: GPU 0]')
parser.add_argument('--batch_size', type=int, default=1, help='Batch Size during training [default: 1]')
parser.add_argument('--num_point', type=int, default=4096, help='Point number [default: 4096]')
parser.add_argument('--model_path', default='/home/hariharan/ur_ws/src/pointnet/src/pointnet/sem_seg/log/model.ckpt', help='model checkpoint file path')
parser.add_argument('--dump_dir', default='/home/hariharan/ur_ws/src/pointnet/src/pointnet/sem_seg/log/dump', help='dump folder path')
parser.add_argument('--output_filelist', default='/home/hariharan/ur_ws/src/pointnet/src/pointnet/sem_seg/log/output_filelist.txt', help='TXT filename, filelist, each line is an output for a room')
parser.add_argument('--room_data_filelist', default='/home/hariharan/ur_ws/src/pointnet/src/pointnet/sem_seg/metal/area6_data_label.txt', help='TXT filename, filelist, each line is a test room data label file.')
parser.add_argument('--no_clutter', action='store_true', help='If true, donot count the clutter class')
parser.add_argument('--visu', default = False, action='store_true', help='Whether to output OBJ file for prediction visualization.')
FLAGS = parser.parse_args()

BATCH_SIZE = FLAGS.batch_size
NUM_POINT = FLAGS.num_point
MODEL_PATH = FLAGS.model_path
GPU_INDEX = FLAGS.gpu
DUMP_DIR = FLAGS.dump_dir
if not os.path.exists(DUMP_DIR): os.mkdir(DUMP_DIR)
LOG_FOUT = open(os.path.join(DUMP_DIR, 'log_evaluate.txt'), 'w')
LOG_FOUT.write(str(FLAGS)+'\n')
ROOM_PATH_LIST = [os.path.join(ROOT_DIR,line.rstrip()) for line in open(FLAGS.room_data_filelist)]

NUM_CLASSES = 13

def log_string(out_str):
    LOG_FOUT.write(out_str+'\n')
    LOG_FOUT.flush()
    #print(out_str)


def eval_one_epoch(sess, ops, room_path, out_data_label_filename, out_gt_label_filename):
    error_cnt = 0
    is_training = False
    total_correct = 0
    total_seen = 0
    loss_sum = 0
    total_seen_class = [0 for _ in range(NUM_CLASSES)]
    total_correct_class = [0 for _ in range(NUM_CLASSES)]
    if FLAGS.visu:
        fout = open(os.path.join(DUMP_DIR, os.path.basename(room_path)[:-4]+'_pred.obj'), 'w')
        fout_gt = open(os.path.join(DUMP_DIR, os.path.basename(room_path)[:-4]+'_gt.obj'), 'w')
    fout_data_label = open(out_data_label_filename, 'w')
    fin_data_label = open(out_data_label_filename, 'r')
    fout_gt_label = open(out_gt_label_filename, 'w')
    current_data, current_label = indoor3d_util.room2blocks_plus_normalized(array_pcd, NUM_POINT, 1.0, 1.0,False, None, 1)
    #current_data, current_label = indoor3d_util.room2blocks_wrapper_normalized(room_path, NUM_POINT)
    current_data = current_data[:,0:NUM_POINT,:]
    current_label = np.squeeze(current_label)
    # Get room dimension..
    data_label = array_pcd
    
    data = data_label[:,0:6]
    max_room_x = max(data[:,0])
    max_room_y = max(data[:,1])
    max_room_z = max(data[:,2])
    
    file_size = current_data.shape[0]
    num_batches = file_size // BATCH_SIZE
    #print(file_size)
    #print(file_size)
    x = []
    y = []
    z = []
    I = []
    #print(num_batches)
    for batch_idx in range(num_batches):
        start_idx = batch_idx * BATCH_SIZE
        end_idx = (batch_idx+1) * BATCH_SIZE
        cur_batch_size = end_idx - start_idx
        
        feed_dict = {ops['pointclouds_pl']: current_data[start_idx:end_idx, :, :],
                     ops['labels_pl']: current_label[start_idx:end_idx],
                     ops['is_training_pl']: is_training}
        loss_val, pred_val = sess.run([ops['loss'], ops['pred_softmax']],
                                      feed_dict=feed_dict)
	
        if FLAGS.no_clutter:
            pred_label = np.argmax(pred_val[:,:,0:12], 2) # BxN
        else:
            pred_label = np.argmax(pred_val, 2) # BxN
        # Save prediction labels to OBJ file
        for b in range(BATCH_SIZE):
            pts = current_data[start_idx+b, :, :]
            l = current_label[start_idx+b,:]
            pts[:,6] *= max_room_x
            pts[:,7] *= max_room_y
            pts[:,8] *= max_room_z
            pts[:,3:6] *= 255.0
            pred = pred_label[b, :]

            for i in range(NUM_POINT):
                color = indoor3d_util.g_label2color[pred[i]]
                color_gt = indoor3d_util.g_label2color[current_label[start_idx+b, i]]
                #if FLAGS.visu:
                    #fout.write('v %f %f %f %d %d %d\n' % (pts[i,6], pts[i,7], pts[i,8], color[0], color[1], color[2]))
                    #fout_gt.write('v %f %f %f %d %d %d\n' % (pts[i,6], pts[i,7], pts[i,8], color_gt[0], color_gt[1], color_gt[2]))
                x.append(pts[i,6])
                y.append(pts[i,7])
                z.append(pts[i,8])
                I.append(pred[i] *50)
                #fout_data_label.write('%f %f %f %d %d %d %f %d\n' % (pts[i,6], pts[i,7], pts[i,8], pts[i,3], pts[i,4], pts[i,5], pred_val[b,i,pred[i]], pred[i]))
                #fout_gt_label.write('%d\n' % (l[i]))
        correct = np.sum(pred_label == current_label[start_idx:end_idx,:])
        total_correct += correct
        total_seen += (cur_batch_size*NUM_POINT)
        loss_sum += (loss_val*BATCH_SIZE)
        for i in range(start_idx, end_idx):
            for j in range(NUM_POINT):
                l = current_label[i, j]
                total_seen_class[l] += 1
                total_correct_class[l] += (pred_label[i-start_idx, j] == l)
	
    #log_string('eval mean loss: %f' % (loss_sum / float(total_seen/NUM_POINT)))
    #log_string('eval accuracy: %f'% (total_correct / float(total_seen)))
    fout_data_label.close()
    fout_gt_label.close()
    
    x = np.array(x)
    predicted  = np.zeros((x.shape[0] , 6))
    predicted  = np.zeros(x.shape[0], dtype=[('x', np.float32),('y', np.float32),('z', np.float32),('I', np.int32)])
    print(x.shape)
    
    min_coordinates = rospy.wait_for_message("/MinPoint" , Point)
    predicted_msg = PointCloud2()
    predicted['x'] = x + min_coordinates.x
    predicted['y'] = np.array(y) + min_coordinates.y
    predicted['z'] = np.array(z) + min_coordinates.z
    predicted['I'] = np.array(I)
    predicted  = np.unique(predicted , axis= 0)
    print(predicted.shape)
    predicted_msg = ros_numpy.msgify(PointCloud2, predicted)
    predicted_msg.header.frame_id = 'camera_link'
    pub = rospy.Publisher("predicted_topic", PointCloud2, queue_size=10)
    pub.publish(predicted_msg)
    #rospy.loginfo("Predicted PointCloud Published")
    
    
    
    
    
    if FLAGS.visu:
        fout.close()
        fout_gt.close()
    return total_correct, total_seen
def CB_array_pcd(data):
    global array_pcd
    global min_x 
    global min_y 
    global min_z 
    np_array_pcd = ros_numpy.numpify(data)
    array_pcd = np.zeros((18000 , 6))
    array_pcd[: , 0] = np_array_pcd['x']
    array_pcd[: , 1] = np_array_pcd['y']
    array_pcd[:, 2] = np_array_pcd['z']
    xyz_min = np.amin(array_pcd, axis=0)[0:3]
    array_pcd[:,0:3] -= xyz_min
    min_x = xyz_min[0]
    min_y = xyz_min[1]
    min_z = xyz_min[2]
    
    
    
    

def callback(data):
    global min_x 
    global min_y 
    global min_z 
    min_x = data.x
    min_y = data.y
    min_z = data.z


def talker():
    with tf.Graph().as_default():
        rospy.init_node("Predictor")
        sub_array_pcd = rospy.Subscriber("/array_pcd" , PointCloud2, CB_array_pcd)
        is_training = False
        with tf.device('/gpu:'+str(GPU_INDEX)):
            pointclouds_pl, labels_pl = placeholder_inputs(BATCH_SIZE, NUM_POINT)
            is_training_pl = tf.placeholder(tf.bool, shape=())
            pred = get_model(pointclouds_pl, is_training_pl)
            loss = get_loss(pred, labels_pl)
            pred_softmax = tf.nn.softmax(pred)
            saver = tf.train.Saver()
            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True
            config.allow_soft_placement = True
            config.log_device_placement = True
            sess = tf.Session(config=config)
            saver.restore(sess, MODEL_PATH)
            log_string("Model restored.")
        ops = {'pointclouds_pl': pointclouds_pl,
        'labels_pl': labels_pl,
        'is_training_pl': is_training_pl,
        'pred': pred,
        'pred_softmax': pred_softmax,
        'loss': loss}
        
        while not rospy.is_shutdown():
            total_correct = 0
            total_seen = 0
            for room_path in ROOM_PATH_LIST:
                out_data_label_filename = os.path.basename(room_path)[:-4] + '_pred.txt'
                out_data_label_filename = os.path.join(DUMP_DIR, out_data_label_filename)
                out_gt_label_filename = os.path.basename(room_path)[:-4] + '_gt.txt'
                out_gt_label_filename = os.path.join(DUMP_DIR, out_gt_label_filename)
        		#print(room_path, out_data_label_filename)
                a, b = eval_one_epoch(sess, ops, room_path, out_data_label_filename, out_gt_label_filename)
                total_correct += a
                total_seen += b
        LOG_FOUT.close()    

if __name__=='__main__':
    try:
        global min_x
        global min_y
        global min_z
        global array_pcd
        talker()
    except rospy.ROSInterruptException:
        pass
