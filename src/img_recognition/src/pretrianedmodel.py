from torchvision import transforms
from torchvision.transforms import ToTensor
from torch.utils.data.dataloader import DataLoader
import torch.nn as nn
import torch
import torch.nn.functional as F
import torchvision
import torchvision.models as models
import torchvision.transforms.functional as TF


transformer = torchvision.transforms.Compose(
    [  # Applying Augmentation
        transforms.ToPILImage(),
        torchvision.transforms.Resize((224, 224)),
        torchvision.transforms.RandomHorizontalFlip(p=0.5),
        torchvision.transforms.RandomVerticalFlip(p=0.5),
        torchvision.transforms.RandomRotation(40),
        torchvision.transforms.ToTensor(),
        torchvision.transforms.Normalize(
        mean=[0.4914, 0.4822, 0.4465], std=[0.2023, 0.1994, 0.2010]
     ),
    ]
)


class PretrainedNet(nn.Module):
    def __init__(self, num_classes=10):
        # Use a pretrained model
        super(PretrainedNet, self).__init__()
        self.network = models.mobilenet_v2()
        # Replace last layer
        #num_ftrs = self.network.fc.in_features
        #self.network.fc = nn.Linear(num_ftrs, 5)
#        for param in self.network.parameters():
#            param.requires_grad = True
#
#        self.network.fc = nn.Linear(512, 5)
# check line 166-169 https://github.com/pytorch/vision/blob/main/torchvision/models/mobilenetv2.py
        self.network.classifier = nn.Sequential(
            nn.Dropout(p=0.2),
            nn.Linear(self.network.last_channel, num_classes),
        )
        
    def forward(self, xb):
        return torch.sigmoid(self.network(xb))

