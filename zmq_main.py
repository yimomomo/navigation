from __future__ import print_function
#from six.moves import xrange
import os
#import better_exceptions
#import tensorflow as tf
import numpy as np
#from tqdm import tqdm
from functools import partial

#from dataset import CitySpaces
#from model import FRRN,_arch_type_a
from webcam_demo import segmentation

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import io
import cv2
import time
from PIL import Image
#from cityscapesScripts.cityscapesscripts.helpers import labels as L

from myzmq.zmq_comm import *
from myzmq.zmq_cfg import *


def get_default_param():
    from datetime import datetime
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return {
        'LOG_DIR':'./log/%s'%(now),

        'TRAIN_NUM' : 100000, #Size corresponds to one epoch
        'BATCH_SIZE': 3,

        'LEARNING_RATE' : 0.001,
        'DECAY_VAL' : 1.0,
        'DECAY_STEPS' : 20000, # Half of the training procedure.
        'DECAY_STAIRCASE' : False,

        'K':512*64,
        'CROP_SIZE':(512,1024),
        'IM_SIZE' :(256,512), #Prediction is made at 1/2 scale.
        'Z_RANGE':0.05,

        'SUMMARY_PERIOD' : 10,
        'SAVE_PERIOD' : 10000,
        'RANDOM_SEED': 0,
    }

class zmq_semanticseg(zmq_comm_svr_c):
    def __init__(self):
        self.initzmqs()
        #self.initnet()
        print('[semanticseg]semantic segmentation init done!')

    def initzmqs(self):
        zmq_comm_svr_c.__init__(self, name=name_semanticseg, ip=ip_semanticseg, port=port_semanticseg)

#    def initnet(self):
#        class MyConfig(dict):
#            pass
#        params = get_default_param()
#        config = MyConfig(params)
#        def as_matrix() :
#            return [[k, str(w)] for k, w in config.items()]
#        config.as_matrix = as_matrix
#   
 #       self.trainId2id = np.zeros((20,),np.uint8)
 #       for i in range(19):
 #           self.trainId2id[i] = L.trainId2label[i].id
#        self.trainId2id[19] = 0
    
#        with tf.variable_scope('params') as params:
#            pass

#        self.im = tf.placeholder(tf.float32,[None,256,512,3])
#        gt = tf.placeholder(tf.int32,[None,256,512])
    
#        with tf.variable_scope('net'):
#            with tf.variable_scope('params') as params:
#                pass
#            self.net = FRRN(None,None,config['K'],self.im,gt,partial(_arch_type_a,20),params,False)

 #       init_op = tf.group(tf.global_variables_initializer(),
#                        tf.local_variables_initializer())
#
#        config = tf.ConfigProto()
#        config.gpu_options.allow_growth = True
 #       self.sess = tf.Session(config=config)
#        self.sess.graph.finalize()
 #       self.sess.run(init_op)
#        MODEL='models/arch_type_a/last.ckpt'
 #       self.net.load(self.sess,MODEL)
#        coord = tf.train.Coordinator()
 #       threads = tf.train.start_queue_runners(coord=coord,sess=self.sess)

    def execute(self,param=None):
        if param is None: 
            print("No Image")
            return
        if isinstance(param,str): param=[param]
        
        res={}
        if(True):#'image' in param):
            st = time.time()
            print('[semanticseg]get an image.')
            img = param#['image']
            img = np.frombuffer(img,dtype=np.uint8)
            img = jpg_to_img_rgb(img)
            orishape = img.shape
            print(orishape)
            img = cv2.resize(img, (480,360))
            input_image = np.asarray([img]).transpose(0,3,1,2)
#            pred = (self.sess.run([self.net.preds], feed_dict={self.im:img}))
#            outim = self.trainId2id[pred][0]
#            outim = cv2.resize(outim, (orishape[1],orishape[0]))
            outim,out_seg_rgb = segmentation(input_image)
            res['seg'] = outim
            #res['rgb'] = out_seg_rgb
            #print(res.type)
            #print(res)
            print('[semanticseg]dealed an image and time is {} seconds.'.format(time.time()-st))

        return list(res.values())[0] if len(res)==1 else res

if __name__ == "__main__":
    segsrv = zmq_semanticseg()
    segsrv.start()
    while(1):
        time.sleep(1)
