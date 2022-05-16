import numpy as np


class PPOMemory:
    def __init__(self, batch_size):
        self.obs_states = []
        self.probs = []
        self.vals = []
        self.actions = []
        self.rewards = []
        self.dones = []
	self.self_states = []
        self.batch_size = batch_size

    def generate_batches(self):
        n_states = len(self.obs_states)
        batch_start = np.arange(0, n_states, 2)
        indices = np.arange(n_states, dtype=np.int64)
        np.random.shuffle(indices)
        batches = [indices[i:i+2] for i in batch_start]

        return self.obs_states,\
            np.array(self.self_states),\
            np.array(self.actions),\
            np.array(self.probs),\
            np.array(self.vals),\
            np.array(self.rewards),\
            np.array(self.dones),\
            batches

    def store_memory(self, obs_state, self_state , action, probs, vals, reward, done):
        self.obs_states.append(obs_state)
        self.self_states.append(self_state)
        self.actions.append(action)
        self.probs.append(probs)
        self.vals.append(vals)
        self.rewards.append(reward)
        self.dones.append(done)

    def clear_memory(self):
        self.obs_states = []
        self.self_states = []
        self.probs = []
        self.actions = []
        self.rewards = []
        self.dones = []
        self.vals = []
