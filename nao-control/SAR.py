class SAR():
    state = [0.0,0.0]
    action = [0.0,0.0]
    reward = 0
    def __init__(self,state=[0.0,0.0],action = [1,1],reward = 0.00000000001):
        self.state = state
        self.action = action
        self.reward = reward
        self.new = True
        return None

    def get(self):
        return self