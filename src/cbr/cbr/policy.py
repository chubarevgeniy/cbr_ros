import torch
import os

class PolicyRunner():
    def __init__(self, path, device='cpu'):
        self.policy = torch.jit.load(os.path.join(path, "policy_script.pt")).to(device)
        self.policy.eval()
        
    def act(self, observations):
        return self.policy(observations)