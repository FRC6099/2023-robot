# Osborn Robotics 2023 Robot - Charged Up
This is the project code for the 2023 Robot. [This is the competition overview](https://www.youtube.com/watch?v=0zpflsYc4PA)

## Contributing
Please use git to clone this repositry if not done so already.
On Windows GitBash is a helpful console to use.

Always branch the latest changes from main by checking out a new branch.
The `workshop` branch will be used during Saturday's workshop, so please
do not touch that branch until then.

During the week, please use a separate branch to push all changes to.

**Helpful Commands:**
```bash
# Checkout main
$ git checkout main

# Get latest changes for checked out branch
$ git pull

# Checkout a new branch (replace "classroom-changes" with your branch name)
$ git checkout -b classroom-changes

# Print status of changes
$ git status

# Stage changes to be committed. Be sure to save all changes before running command
$ git add .

# Be sure to print status to see what is staged before committing
# Commit changes
$ git commit -m "A message of what I did"

# Push changes to GitHub (replace "classroom-changes" with your branch name)
$ git push origin classroom-changes
```

## Structure

- `Robot` class - Core class which is auto-generated
- `RobotContainer` class - Where all `subsystems`, `commands`, and `controllers` are initialized.
- `commands` package - Contains all the classes to perform a teleop or autonomouus behavior.
- `controllers` package - Contains the classes that `commands` classes use to retrieve non-boolean input (e.g. joysticks or triggers).
- `subsystems` package - Contains all the classes that interact with motors and various sensors.
