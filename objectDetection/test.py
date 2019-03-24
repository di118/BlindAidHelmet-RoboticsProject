from __future__ import absolute_import, division, print_function
import tensorflow.keras.backend as K
import numpy as np
import PIL.Image as Image
import matplotlib.pylab as plt

import tensorflow as tf
import tensorflow_hub as hub

from tensorflow.keras import layers

classifier_url = "https://tfhub.dev/google/imagenet/mobilenet_v2_100_224/classification/2"



def classifier(x):
    classifier_module = hub.Module(classifier_url)
    return classifier_module(x)


IMAGE_SIZE = hub.get_expected_image_size(hub.Module(classifier_url))

image_data = image_generator.flow_from_directory(str(data_root), target_size=IMAGE_SIZE)
for image_batch,label_batch in image_data:
  print("Image batch shape: ", image_batch.shape)
  print("Labe batch shape: ", label_batch.shape)
  break

sess = K.get_session()
init = tf.global_variables_initializer()

sess.run(init)


grace_hopper = tf.keras.utils.get_file('image.jpg','https://storage.googleapis.com/download.tensorflow.org/example_images/grace_hopper.jpg')
grace_hopper = Image.open(grace_hopper).resize(IMAGE_SIZE)
grace_hopper

grace_hopper = np.array(grace_hopper)/255.0
grace_hopper.shape
result = classifier_model.predict(grace_hopper[np.newaxis, ...])
result.shape
predicted_class = np.argmax(result[0], axis=-1)
predicted_class