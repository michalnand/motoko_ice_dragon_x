import numpy

class ModelInference:

    def __init__(self, params):
        self.W_ih = params['W_ih']  # (3*hidden, input)
        self.W_hh = params['W_hh']  # (3*hidden, hidden)
        self.b_ih = params['b_ih']  # (3*hidden,)
        self.b_hh = params['b_hh']  # (3*hidden,)

        self.W_out = params("W_out")
        self.b_out = params("b_out")


    def forward(self, x, h):

        gate_x = x @ self.W_ih.T + self.b_ih
        gate_h = h @ self.W_hh.T + self.b_hh

        x_z, x_r, x_n = numpy.split(gate_x, 3, axis=1)
        h_z, h_r, h_n = numpy.split(gate_h, 3, axis=1)

        z_gate  = self._sigmoid(x_z + h_z)
        r_gate  = self._sigmoid(x_r + h_r)
        n_gate  = numpy.tanh(x_n + r_gate * h_n)

        h_new   = (1 - z_gate) * n_gate + z_gate * h
        y       = h_new @ self.W_out.T + self.b_out

        return y, h_new
