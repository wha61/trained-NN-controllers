import torch
import torch.nn as nn

class SimpleModel(nn.Module):
    def __init__(self):
        super(SimpleModel, self).__init__()
        self.layer_0 = nn.Linear(18, 64)
        self.layer_1 = nn.Linear(64, 64)
        self.layer_2 = nn.Linear(64, 4)

    def forward(self, x):
        x = self.layer_0(x)
        x = self.layer_1(x)
        x = self.layer_2(x)
        return x

model = SimpleModel()
torch.save(model.state_dict(), 'model_test.pt')
