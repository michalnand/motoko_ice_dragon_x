from dataset    import *
from model      import *




if __name__ == "__main__":
    n_samples   = 100000

    n_epoch     = 5
    batch_size  = 256
    seq_len     = 256

    n_inputs    = 1
    n_outputs   = 1
    n_hidden    = 16

    

    training_dataset = LineDataset(n_samples)

    
    #model = RNNGRUModel(n_inputs, n_outputs, n_hidden)
    model = RNNBasicModel(n_inputs, n_outputs, n_hidden)
    print(model)

    optimizer = torch.optim.AdamW(model.parameters(), lr=0.01)

    steps = n_epoch*len(training_dataset)//batch_size
    for n in range(steps):

        x, y_gt = training_dataset.get_batch(batch_size, seq_len)

        x = x + 0.1*torch.randn_like(x)

        y_pred = model(x)

        tv = y_pred[:, 1:, :] - y_pred[:, :-1, :]
        tv_loss = torch.mean(torch.abs(tv)) 
        mse_loss = ((y_gt.detach() - y_pred)**2).mean()

        loss = mse_loss + 0.1*tv_loss

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        print(n, loss, round(100.0*n/steps, 2))


    #torch.save(model.state_dict(),  "trained/model_gru_" + str(n_hidden) + ".pt")
    torch.save(model.state_dict(),  "trained/model_rnn_" + str(n_hidden) + ".pt")