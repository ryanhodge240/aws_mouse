In this folder, you will see the recording.yaml and two deep learning nernual network models generated from AWS SageMaker (pytorchimage)

In the recording.yaml file, you see line 1 (flower.pth) and line 23 (flower_pretrainm2_gpu.pth)

You will need to include the label index in lines 4-8 and lines 26-30.

Also, line 18 show model (flower.pth is a custom model) for custom and line 40 for pretrained model (flower_pretrainm2_gpu.pth is from mobilenet) 


When the ROS node is launched, img_recognition/param/inference_model.yaml shows which model will be used. You will need to specify a model name in line 1 that appears
in recording.yaml file.
