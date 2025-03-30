import torch

class RNNGRUModel(torch.nn.Module):
    def __init__(self, n_inputs, n_outputs, n_hidden):
        super(RNNGRUModel, self).__init__()

        self.rnn = torch.nn.GRU(n_inputs, n_hidden, batch_first=True)
        self.lin = torch.nn.Linear(n_hidden, n_outputs)

        torch.nn.init.orthogonal_(self.rnn.weight_hh_l0, 0.5)
        torch.nn.init.orthogonal_(self.rnn.weight_ih_l0, 0.5)
        torch.nn.init.zeros_(self.rnn.bias_hh_l0)
        torch.nn.init.zeros_(self.rnn.bias_ih_l0)

        torch.nn.init.orthogonal_(self.lin.weight, 0.01)
        torch.nn.init.zeros_(self.lin.bias)


    def forward(self, x):
        z, _ = self.rnn(x)
        y = self.lin(z)
                     
        return y
    
    def extract_numpy_params(self):
        return {
            "W_ih": self.rnn.weight_ih_l0.detach().float().cpu().numpy(),  # (3*hidden, input)
            "W_hh": self.rnn.weight_hh_l0.detach().float().cpu().numpy(),  # (3*hidden, hidden)
            "b_ih": self.rnn.bias_ih_l0.detach().float().cpu().numpy(),    # (3*hidden,)
            "b_hh": self.rnn.bias_hh_l0.detach().float().cpu().numpy(),    # (3*hidden,)
            "W_out": self.lin.weight.detach().float().cpu().numpy(),       # (output, hidden)
            "b_out": self.lin.bias.detach().float().cpu().numpy()          # (output,)
        }
    


class RNNBasicModel(torch.nn.Module):
    def __init__(self, n_inputs, n_outputs, n_hidden):
        super(RNNBasicModel, self).__init__()

        self.rnn = torch.nn.RNN(n_inputs, n_hidden, batch_first=True)
        self.lin = torch.nn.Linear(n_hidden, n_outputs)

        torch.nn.init.orthogonal_(self.lin.weight, 0.01)
        torch.nn.init.zeros_(self.lin.bias)


    def forward(self, x):
        z, _ = self.rnn(x)
        y = self.lin(z)
                     
        return y
  

if __name__ == "__main__":

    batch_size = 10
    n_inputs   = 7
    n_hidden   = 64
    n_outputs  = 3

    seq_len = 200

    model = RNNModel(n_inputs, n_outputs, n_hidden)

    x = torch.randn((batch_size, seq_len, n_inputs))

    model(x)

        