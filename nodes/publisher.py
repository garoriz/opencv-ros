#!/usr/bin/env python3
# encoding: utf-8

import rospy
import numpy as np
from sensor_msgs.msg import Image

from typing import Final

# constants
ROS_NODE_NAME: Final[str] = "publisher"

ROS_PARAM_PUB_RATE: Final[int] = 30
ROS_IMAGE_TOPIC: Final[str] = "image"

def generate_image(width: int = 320, height: int = 240) -> Image:
    img = np.random.randint(0, 256, (height, width), dtype=np.uint8)

    image_msg = Image()
    image_msg.height = height
    image_msg.width = width
    image_msg.encoding = "mono8"
    image_msg.step = width
    image_msg.data = img.tobytes()

    return image_msg

def main() -> None:
  rospy.init_node(ROS_NODE_NAME)

  pub_frequency: int = rospy.get_param("~rate", ROS_PARAM_PUB_RATE)

  # Q: Почему здесь не нужно писать rospy.resolve_name(ROS_IMAGE_TOPIC)?
  # A: ROS автоматически преобразует имена топиков
  publisher = rospy.Publisher(ROS_IMAGE_TOPIC, Image, queue_size=10)

  # Обратите внимание: топик "image" может переименоваться при запуске ROS-узла.
  # rosrun project_template publisher.py image:=image_raw
  # Более подробно об этом можно узнать по ссылке: http://wiki.ros.org/Names
  rospy.loginfo(f"Publishing to '{rospy.resolve_name(ROS_IMAGE_TOPIC)}' at {pub_frequency} Hz ...")

  rate = rospy.Rate(pub_frequency)

  while not rospy.is_shutdown():
    # Задание 1: сгенерируйте случайное изображение.
    # Разрешение: 320 x 240 (ширина x высота).
    # Формат пикселей: монохром, 8-бит.
    # Создайте функцию для генерации изображения "generate_image(width = 320, height = 240)".
    image_msg = generate_image()
    publisher.publish(image_msg)
    
    rate.sleep()


if __name__ == '__main__':
    main()
