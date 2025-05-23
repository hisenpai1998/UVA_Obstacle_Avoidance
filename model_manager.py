import torch
import torch.nn     as nn
import numpy        as np
from   skfuzzy      import cmeans

class Attention(nn.Module):
    def __init__(self, hidden_dim):
        super(Attention, self).__init__()
        self.attn = nn.Linear(hidden_dim, 1)
        self.softmax = nn.Softmax(dim=1)

    def forward(self, x):
        attn_weights = self.softmax(self.attn(x))
        context = torch.sum(attn_weights * x, dim=1)
        return context

class UAVNavigationModel(nn.Module):
    def __init__(self, num_classes=9, n_clusters_list=[3, 5], hidden_dim=256):
        super(UAVNavigationModel, self).__init__()
        self.total_clusters = sum(n_clusters_list)
        self.hidden_dim = hidden_dim

        self.cnn = nn.Sequential(
            nn.Conv2d(self.total_clusters, 64, kernel_size=3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.MaxPool2d(2),

            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(),
            nn.MaxPool2d(2),

            nn.AdaptiveAvgPool2d((1, 1))
        )

        self.lstm = nn.LSTM(128, self.hidden_dim, batch_first=True)

        self.attention = Attention(self.hidden_dim)
        
        self.fc = nn.Sequential(
            nn.Linear(self.hidden_dim, 128),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(128, num_classes)
        )

    def forward(self, x):
        batch_size, seq_len, c, h, w = x.size()
        cnn_features = []
        for t in range(seq_len):
            frame = x[:, t, :, :, :]
            cnn_out = self.cnn(frame)
            cnn_out = cnn_out.view(batch_size, -1)
            cnn_features.append(cnn_out)
        cnn_features = torch.stack(cnn_features, dim=1)
        lstm_out, _ = self.lstm(cnn_features)
        context = self.attention(lstm_out)
        output = self.fc(context)
        return output

def apply_fcm(depth_image, n_clusters_list=[3, 5]):
    depth_image = (depth_image - depth_image.min()) / (depth_image.max() - depth_image.min() + 1e-8)
    data = depth_image.reshape(-1, 1).astype(np.float64)
    segmented = []
    for n_clusters in n_clusters_list:
        cntr, u, *_ = cmeans(data.T, c=n_clusters, m=2, error=0.005, maxiter=1000)
        u_reshaped = u.reshape(n_clusters, *depth_image.shape)
        segmented.append(u_reshaped)
    return np.concatenate(segmented, axis=0)