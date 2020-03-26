# TRGAIL
## Instructions

### PPO (Pusher)
``` bash
python run_ppo.py --env_name Pusher-v2 
```

### GAIL (Pusher, 15epis)
``` bash
python run_gail.py --env_name Pusher-v2 --expert_fname Pusher-v2_ppo_15epis.pkl
``` 

### TRGAIL (Pusher, 15epis)
``` bash
python run_gail.py --env_name Pusher-v2 --expert_fname Pusher-v2_ppo_15epis.pkl --tr
``` 

## Reference
https://github.com/DeepX-inc/machina