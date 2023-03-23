library(tidyverse)
library(here)
library(readxl)

data <- read_xlsx(here::here("OneDrive - Cal Poly","AERO553","readings.xlsx"))

data_clean <- data |>
  select(-accelKFprop, -acoustKFprop, -combKFprop) |>
  rename(
    `Accel. Pos. (raw)` = accel,
    `Accel. Pos. (KF)` = accelKF,
    `Acoustic (raw)` = acoust,
    `Acoustic (KF)` = acoustKF,
    `Combined KF` = combinedKF
  ) |> 
  pivot_longer(cols = !x, names_to = "pos_est", values_to = "value")

data_clean2 <- data |>
  select(x, acoust, acoustKF) |>
  rename(
    `Acoustic (raw)` = acoust,
    `Acoustic (KF)` = acoustKF,
  ) |> 
  pivot_longer(cols = !x, names_to = "pos_est", values_to = "value")

data_props <- data |> 
  select(x,
         accelKFprop,
         acoustKFprop,
         combKFprop) |> 
  rename(
    `Accel. Pos. (KF)` = accelKFprop,
    `Acoustic (KF)` = acoustKFprop,
    `Combined KF` = combKFprop
  ) |> 
  pivot_longer(cols = !x, names_to = "method", values_to = "proportion") |> 
  mutate(proportion = proportion*100)

data_props |> 
  ggplot(mapping= aes(x = x, y = proportion, color = method)) +
  geom_line() +
  labs(title = "MMAEKF Output proportions",
       subtitle = "Proportion (%)",
       x = "Actual Distance (mm)",
       y = "",
       color = "Method") +
  xlim(40, 120)
  
data_clean |> 
  ggplot(mapping = aes(x = x, y = value, color = pos_est)) +
  geom_line() +
  labs(title = "Position Estimation Comparison",
       subtitle = "Estimated Distance (mm)",
       x = "Actual Distance (mm)",
       y = "",
       color = "Method") +
  geom_abline(linetype = "dashed") +
  ylim(-80, 600) +
  xlim(40, 120)

data_clean2 |> 
  ggplot(mapping = aes(x = x, y = value, color = pos_est)) +
  geom_line() +
  labs(title = "Position Estimation Comparison",
       subtitle = "Estimated Distance (mm)",
       x = "Actual Distance (mm)",
       y = "",
       color = "Method") +
  geom_abline(linetype = "dashed") +
  xlim(40, 120)

stationary_data <- read_xlsx(here::here("OneDrive - Cal Poly","AERO553","stationary.xlsx")) |> 
  filter(x == 82.78) |> 
  mutate(n = row_number()) |> 
  select(x, n, acoust, acoustKF) |> 
  rename(`Acoustic (raw)` = acoust,
         `Acoustic (KF)` = acoustKF) |> 
  pivot_longer(cols = c(`Acoustic (raw)`,`Acoustic (KF)`), names_to = "Method", values_to = "Measurement")
stationary_data |> 
  ggplot(mapping = aes(x = n, y = Measurement, color = Method)) +
  labs(title = "Stationary Position Estimation Comparison",
       subtitle = "Estimated Distance (mm) when held at 83mm",
       x = "Sample",
       y = "",
       color = "Method") +
  geom_line() +
  geom_abline(slope = 0, intercept = 82.78, linetype = "dashed")
