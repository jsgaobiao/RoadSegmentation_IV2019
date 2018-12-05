import matplotlib.pyplot as plt
import numpy as np
import cv2
import tensorflow as tf
import TensorflowUtils as utils
import read_Data as image_reader
import datetime
import BatchDatsetReader as dataset
from tensorflow.python.framework import ops

FLAGS = tf.flags.FLAGS
tf.flags.DEFINE_integer("batch_size", "1", "batch size for training")
tf.flags.DEFINE_float("alpha", "0.1", "L2 regularization parameter, alpha")
tf.flags.DEFINE_float("learning_rate", "1e-4", "Learning rate for Adam Optimizer")
tf.flags.DEFINE_string("data_dir", "../../data/data_easy/", "path to dataset")
tf.flags.DEFINE_string("logs_dir", "../../results/svm/checkpoints/", "path to logs directory")
tf.flags.DEFINE_string("vis_dir", "../../results/svm/vis/", "path to store visualization results")
tf.flags.DEFINE_string('mode', "train", "Mode train/ test/ visualize")

NUM_OF_FEATURE = 8
IMAGE_HEIGHT = 300
IMAGE_WIDTH = 300
MAX_ITERATION = int(1e4 + 1)

# visualization prediction image
def VisPred(pred, imgs):
    ret = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH))
    cnt = 0
    for i in xrange(IMAGE_HEIGHT):
        for j in xrange(IMAGE_WIDTH):
            if (imgs[0, i, j, 0] > 0):
                if (pred[cnt] > 0):
                    ret[i, j] = 1
                else:
                    ret[i, j] = 2
                cnt += 1
    cv2.imwrite(FLAGS.vis_dir + "test.png", ret)

# main

ops.reset_default_graph()

# Initialize placeholders
x_data = tf.placeholder(shape=[None, NUM_OF_FEATURE], dtype=tf.float32, name="input_feature")
y_label = tf.placeholder(shape=[None, 1], dtype=tf.float32, name="annotation")

# Create variables for linear regression
A = tf.Variable(tf.random_normal(shape=[NUM_OF_FEATURE, 1]))
b = tf.Variable(tf.random_normal(shape=[1, 1]))

# Declare model operations
model_output = tf.subtract(tf.matmul(x_data, A), b)

# Declare vector L2 'norm' function squared
l2_norm = tf.reduce_sum(tf.square(A))

# Declare loss function
# Loss = max(0, 1-pred*actual) + alpha * L2_norm(A)^2
# L2 regularization parameter, alpha
alpha = tf.constant(float(FLAGS.alpha))
# Margin term in loss
classification_term = tf.reduce_mean(tf.maximum(0., tf.subtract(1., tf.multiply(model_output, y_label))))
# Put terms together
loss = tf.add(classification_term, tf.multiply(alpha, l2_norm))

# Declare prediction function
prediction = tf.sign(model_output)
accuracy = tf.reduce_mean(tf.cast(tf.equal(prediction, y_label), tf.float32))

# Declare optimizer
my_opt = tf.train.GradientDescentOptimizer(FLAGS.learning_rate)
train_op = my_opt.minimize(loss)

# Create graph
sess = tf.Session()
# Initialize variables
sess.run(tf.global_variables_initializer())

print("Setting up Model Saver...")
saver = tf.train.Saver(max_to_keep = 10)
ckpt = tf.train.get_checkpoint_state(FLAGS.logs_dir)
if ckpt and ckpt.model_checkpoint_path:
    saver.restore(sess, ckpt.model_checkpoint_path)
    print("Model restored...")
    print("and path is : " + ckpt.model_checkpoint_path)

# Get data
train_records, valid_records, test_records = image_reader.read_dataset(FLAGS.data_dir)
print('Train num:', len(train_records))
print('Test num:', len(test_records))
print("Setting up dataset reader")
image_options = {'resize': False, 'resize_size': 300}
train_dataset_reader = dataset.BatchDatset(train_records, image_options)
test_dataset_reader = dataset.BatchDatset(test_records, image_options)

if (FLAGS.mode == "train"):
    # Training loop
    loss_vec = []
    train_accuracy = []
    test_accuracy = []
    for i in xrange(MAX_ITERATION):
        train_vector, train_annotations, trains_images = train_dataset_reader.next_batch(FLAGS.batch_size, random_rotate = 0)
        sess.run(train_op, feed_dict={x_data: train_vector, y_label: train_annotations})

        temp_loss = sess.run(loss, feed_dict={x_data: train_vector, y_label: train_annotations})
        loss_vec.append(temp_loss)

        train_acc_temp = sess.run(accuracy, feed_dict={x_data: train_vector, y_label: train_annotations})
        train_accuracy.append(train_acc_temp)

        test_vector, test_annotations, test_images = test_dataset_reader.next_batch(FLAGS.batch_size, random_rotate = 0)
        test_acc_temp = sess.run(accuracy, feed_dict={x_data: test_vector, y_label: test_annotations})
        test_accuracy.append(test_acc_temp)

        if ((i + 1) % 100 == 0):
            print('Step #{} A = {}, b = {}'.format(str(i+1), str(sess.run(A)), str(sess.run(b))))
            print('Loss = ' + str(temp_loss))
        
        if ((i + 1) % 500 == 0):
            saver.save(sess, FLAGS.logs_dir + "model.ckpt", i)

        # Visualize an example in test
        test_pred = sess.run(prediction, feed_dict={x_data: test_vector, y_label: test_annotations})
        VisPred(test_pred, test_images)
