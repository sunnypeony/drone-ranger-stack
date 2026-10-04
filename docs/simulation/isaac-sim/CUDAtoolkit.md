
#### CUDA Toolkit Install

##### Step 1 - Uninstall Ubuntu built-in CUDA toolkit

```bash
sudo apt remove --purge -y nvidia-cuda-toolkit
sudo apt autoremove -y
```

##### Step 2 - Get Official CUDA Toolkit

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb

sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update

sudo apt install -y cuda-toolkit-12-4
```
##### Step 3 - Set Environmental Variable
```bash
natasha@dronerangerlab:~$ echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

##### Step 4 - Verify
```bash
natasha@dronerangerlab:~$ which nvcc
/usr/local/cuda/bin/nvcc

natasha@dronerangerlab:~$ nvcc --version
nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2024 NVIDIA Corporation
Built on Thu_Mar_28_02:18:24_PDT_2024
Cuda compilation tools, release 12.4, V12.4.131
Build cuda_12.4.r12.4/compiler.34097967_0
```
