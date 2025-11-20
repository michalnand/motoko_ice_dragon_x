from dataset    import *
from model      import *
import matplotlib.pyplot as plt




if __name__ == "__main__":
    n_samples   = 2000

    n_epoch     = 5
    batch_size  = 64
    seq_len     = 256

    n_inputs    = 1
    n_outputs   = 1
    n_hidden    = 8

    

    testing_dataset = LineDataset(n_samples)


    model = RNNGRUModel(n_inputs, n_outputs, n_hidden)
    #model = RNNBasicModel(n_inputs, n_outputs, n_hidden)
    
    model.load_state_dict(torch.load("trained/model_gru_" + str(n_hidden) + ".pt"))

    print(model)

    print(testing_dataset.x_result.shape, testing_dataset.y_result.shape)

    x = testing_dataset.x_result
    x = x.unsqueeze(0).unsqueeze(2)
    y_pred = model(x)

    x_ref  = testing_dataset.x_result.detach().cpu().numpy()
    yc_ref = testing_dataset.y_result.detach().cpu().numpy()
    yc_pred= y_pred[0, :, 0].detach().cpu().numpy()


    plt.plot(x_ref, color="gray")
    plt.plot(yc_ref, color="red")
    plt.plot(yc_pred, color="purple")
    plt.show()
