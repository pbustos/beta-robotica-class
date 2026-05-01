import gymnasium as gym
import ale_py
import numpy as np
import pygame 

gym.register_envs(ale_py)

def extract_features(ram_obs):
    """Extract interpretable features from Pong RAM."""
    ball_x    = ram_obs[49]
    ball_y    = ram_obs[54]
    player_y  = ram_obs[51]  # Right paddle (you)
    enemy_y   = ram_obs[50]  # Left paddle (opponent)

    features = np.array([
        ball_x / 255.0,
        ball_y / 255.0,
        player_y / 255.0,
        enemy_y / 255.0,
    ], dtype=np.float32)

    return features

def rule_based_action(features):
    """Simply chase the ball with the player paddle."""
    ball_y   = features[1]
    player_y = features[2]

    dead_zone = 0.02

    if player_y < ball_y - dead_zone:
        return 3  # Move DOWN
    elif player_y > ball_y + dead_zone:
        return 2  # Move UP
    else:
        return 0  # NOOP

def run_episode(env):
    obs, _ = env.reset()
    total_reward = 0

    while True:
        features = extract_features(obs)
        action = rule_based_action(features)
        obs, reward, terminated, truncated, _ = env.step(action)
        total_reward += reward

        if terminated or truncated:
            break

    return total_reward

# --- Main ---
env = gym.make("ALE/Pong-v5", render_mode="rgb_array", obs_type="ram")
obs, _ = env.reset()

pygame.init()
screen = pygame.display.set_mode((500, 500), pygame.RESIZABLE)  # half the default size
clock = pygame.time.Clock()

while True:
    features = extract_features(obs)
    action = rule_based_action(features)
    obs, reward, terminated, truncated, _ = env.step(action)
    
    frame = env.render()  # returns (210, 160, 3) numpy array
    surface = pygame.surfarray.make_surface(frame.transpose(1, 0, 2))

    win_size = screen.get_size()
    surface = pygame.transform.scale(surface, win_size)
    screen.blit(surface, (0, 0))
    pygame.display.flip()
    clock.tick(60)  # 60 fps cap
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            env.close()
            pygame.quit()
            exit()
    
    if terminated or truncated:
        obs, _ = env.reset()

env.close()
