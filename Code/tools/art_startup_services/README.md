# Running services

## Setup

This will create an environment file that contains `ROS_DISTRO` and `RACE_COMMON_DIR` those will be used to run the services, it will be stored in /etc/default/race_common.
It also creates symbolic links to the user folder `$HOME/.config/systemd/user/` for each service.

```bash
bash ./src/tools/art_startup_services/setup.bash
```

## Start services

```bash
systemctl --user start <service_name>
```

## Stop services

```bash
systemctl --user start <service_name>
```

## See the status of the services

```bash
systemctl --user status <service_name>
```