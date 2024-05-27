import tempfile

import mlpfile
import mlpfile.torch
import torch

N_HIDDEN = 1
HIDDEN_DIM = 32
XDIM = 15
UDIM = 4

def main():
    HIDDEN = [HIDDEN_DIM] * N_HIDDEN
    mlp = mlpfile.torch.mlp(XDIM, UDIM, HIDDEN)
    with torch.no_grad():
        for p in mlp.parameters():
            p *= 0

    with tempfile.NamedTemporaryFile() as f:
        mlpfile.torch.write(mlp, f.name)
        model = mlpfile.Model.load(f.name)

    mlpfile.codegen(model, outdir=".", eigen=True, compile=False)


if __name__ == "__main__":
    main()
