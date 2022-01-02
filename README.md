# Model-Predictive-Perimeter-Control-with-multi-region-MFDs

## Usage

This is a model predictive perimeter control simulation with multi region MFDs.

It requires 'mpctools' and 'CasADi', both can be acquired [here](https://bitbucket.org/rawlings-group/octave-mpctools/src/master/)

Simply execute the 'run.m' file to run.

----------------------------------------------------------------------------------------------------------------------------------------------------

## Setup

The setup is as follows.

The heterogenous urban network consists of two regions with the dynamics of a specified macroscopic fundamental diagram (MFD).

<img src='https://user-images.githubusercontent.com/28818052/147873745-d92c8b7f-fea0-4053-8490-3e1ea72bc740.png' width='500'>

The dynamics of this network is described as bellow.

<img src='https://user-images.githubusercontent.com/28818052/147873747-0641d412-b5f5-4f93-812f-955ad5926a76.png' width='500'>
<img src='https://user-images.githubusercontent.com/28818052/147873748-ddcf1d1e-cfe2-4e6f-b0d4-c38e788d423b.png' width='500'>

Here, 'n' is the accumulation, 'q' is the demand inflow, and 'u' is the perimeter control parameter. 'G' represents the MFD.

Further constraints are decribed bellow.

<img src='https://user-images.githubusercontent.com/28818052/147873749-d105c461-9229-4bcc-83ba-491012ad2e22.png' width='600'>

The specific values are adopted from ['Nikolas Geroliminis, Jack Haddad, and Mohsen Ramezani. Optimal perimeter control for two urban regions with macroscopic fundamental diagrams: A model predictive approach. IEEE Transactions on Intelligent Transportation Systems, 14(1): 348–359, 2013'](https://ieeexplore.ieee.org/document/6353591) with some simplications.

## Formulation

In order to find the perimeter control parameter that minimizes the total time spent (TTS) for a finite horizon, the following discrete-time economic non-linear MPC problem is formulated.

<img src='https://user-images.githubusercontent.com/28818052/147874088-2d1f230d-5d49-433a-b837-1a6d769dbe42.png' width='600'>

## Results for Nc = 20

The following figures show the results of the simulation when the prediction horizon Nc is equal to 20.

![Figure_1](https://user-images.githubusercontent.com/28818052/147874145-747626bd-244f-4092-925f-dbe3807edd88.png)
![Figure_2](https://user-images.githubusercontent.com/28818052/147874146-7f8add8c-0c2e-487c-be5b-95bad0f5dfe6.png)
![Figure_3](https://user-images.githubusercontent.com/28818052/147874147-c21bc4c1-f248-49cc-9ab0-955f616ba988.png)
![Figure_4](https://user-images.githubusercontent.com/28818052/147874148-e2455d8e-9962-48f0-90d4-5218c8bb7573.png)
![Figure_5](https://user-images.githubusercontent.com/28818052/147874149-90513035-4351-4569-862c-28d1b0d9b235.png)

## TTS for different Nc values

The following figure shows the total time spent per simulation with different values of Nc (1 ~ 40).

![Figure_6](https://user-images.githubusercontent.com/28818052/147874182-21d2bf05-0ea8-481e-82fc-923031e075a5.png)

Here, any results before Nc = 17 can be ignored as they result in a gridlock where maximum accumulation is reached and no outflow is possible.

As for Nc > 17, in creasing the value of Nc significantly decreases the TTS.

However, after a certain point (around Nc > 30), having longer horizons does not reward more information for the MPC model, and therefore the curve flattens out.
