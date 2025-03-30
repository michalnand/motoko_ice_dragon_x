import numpy
import matplotlib.pyplot as plt

import torch

class LineDataset:

    def __init__(self, n_samples):
        x_result = numpy.zeros(50)
        while x_result.shape[0] < n_samples:            
            behavior_type = numpy.random.choice(["straight", "offset", "oscillate", "damped_oscillate", "raise_oscillate"])
            dx, x, y = self._generate_line_segment(behavior_type = behavior_type)

            x_result = self._smooth_transition(x_result, y)

        x_result = torch.from_numpy(x_result).float()
        y_result = self._anotate(x_result).float()

        len = min(x_result.shape[0], y_result.shape[0])

        self.x_result = x_result[0:len]
        self.y_result = y_result[0:len]

    def __len__(self):
        return self.x_result.shape[0]

    '''
    def get_batch(self, batch_size, seq_len = 256, w_size = 16):
        x_result = torch.zeros((batch_size, seq_len, 1))
        y_result = torch.zeros((batch_size, seq_len, 2))

        for n in range(batch_size):
            ofs_start = numpy.random.randint(0, self.x_result.shape[0] - seq_len - w_size - 1)
            ofs_end   = ofs_start + seq_len

            x_result[n, :, 0] = self.x_result[ofs_start:ofs_end]
            y_result[n, :, 0] = self.y_result[ofs_start:ofs_end]
            y_result[n, :, 1] = self.y_result[w_size + ofs_start:w_size + ofs_end]

        return x_result, y_result
    '''

    def get_batch(self, batch_size, seq_len = 256):
        x_result = torch.zeros((batch_size, seq_len, 1))
        y_result = torch.zeros((batch_size, seq_len, 1))

        for n in range(batch_size):
            ofs_start = numpy.random.randint(0, self.x_result.shape[0] - seq_len - 1)
            ofs_end   = ofs_start + seq_len

            x_result[n, :, 0] = self.x_result[ofs_start:ofs_end]
            y_result[n, :, 0] = self.y_result[ofs_start:ofs_end]

        return x_result, y_result

    def _generate_line_segment(self, step_mean=5.0, step_std=0.5, behavior_type='straight'):

        n_samples = numpy.random.randint(20, 100)
    
        # Generate cumulative distance x
        dx = numpy.random.normal(loc=step_mean, scale=step_std, size=n_samples)
        x = numpy.cumsum(dx)

        if behavior_type == 'straight':
            y = numpy.zeros_like(x)

        elif behavior_type == 'offset':
            offset = numpy.random.choice([-1, 1])*numpy.random.uniform(0.0, 1.0)
            y = numpy.zeros_like(x)
            y[:] = offset

        elif behavior_type == 'oscillate':
            A       = numpy.random.uniform(0.05, 1.0)
            lam     = numpy.random.uniform(10.0, 300.0)
            w       = 2.0*numpy.pi / lam
            phase   = numpy.random.uniform(0, 2.0 * numpy.pi)
            y       = A*numpy.sin(w*x + phase)

        elif behavior_type == 'damped_oscillate':
            A0  = numpy.random.uniform(0.05, 1.0)
            lam = numpy.random.uniform(10.0, 150.0)
            w   = 2 * numpy.pi / lam
            phase = numpy.random.uniform(0, 2.0 * numpy.pi)

            a = numpy.random.uniform(0.005, 0.02)
            b = numpy.random.uniform(0.003, 0.015)

            A = A0 * numpy.exp(-a * x)
            w = w * numpy.exp(-b * x)
            y = A * numpy.sin(w * x + phase)

        elif behavior_type == 'raise_oscillate':
            A0  = numpy.random.uniform(0.05, 1.0)
            lam = numpy.random.uniform(10.0, 150.0)
            w   = 2 * numpy.pi / lam
            phase = numpy.random.uniform(0, 2.0 * numpy.pi)

            a = numpy.random.uniform(0.005, 0.02)
            b = numpy.random.uniform(0.003, 0.015)

            A = A0 * (1.0 - numpy.exp(-a * x))
            w = w  * (1.0 - numpy.exp(-b * x))
            y = A  * numpy.sin(w * x + phase)



        else:
            raise ValueError(f"Unknown behavior_type: {behavior_type}")

        y = numpy.clip(y, -1.0, 1.0)
        return dx, x, y
    

    def _smooth_transition(self, y1, y2, overlap = 20):
        assert len(y1) >= overlap and len(y2) >= overlap
        alpha = 0.5 * (1 - numpy.cos(numpy.linspace(0, numpy.pi, overlap)))

        y_blend = (1 - alpha) * y1[-overlap:] + alpha * y2[:overlap]
        y_combined = numpy.concatenate([
            y1[:-overlap],
            y_blend,
            y2[overlap:]
        ])

        return y_combined
    

    def _anotate(self, x, window_size = 16):
        pool = torch.nn.MaxPool1d(window_size, stride=1, padding=window_size//2)

        result = pool(torch.abs(x).unsqueeze(0))

        return result.squeeze(0)

if __name__ == "__main__":

    dataset = LineDataset(1000)

    x, y = dataset.get_batch(128)

    print(x.shape, y.shape)

    x = x[0, :, 0].detach().cpu().numpy()
    yc = y[0, :, 0].detach().cpu().numpy()
    yf = y[0, :, 1].detach().cpu().numpy()


    plt.plot(x)
    plt.plot(yc)
    plt.plot(yf)
    plt.show()