## **Upload your `.md` file to GitHub**


###  1. Clone the Repository Locally

Open Git Bash or a terminal, then run:


```bash
cd "C:\Users\DOPPS\Desktop\ROS Object"
git clone https://github.com/ArshithaRajkumar/ROS-Docker.git
```

> This will create a folder:

```
C:\Users\DOPPS\Desktop\ROS Object\ROS-Docker
```




### 2. **Open Git Bash**, then run:

```bash
cd "/c/Users/DOPPS/Desktop/ROS Object/<the corresponding file>"
```

### 3. **Check Git status**:

```bash
git init
git status
```

You should see your markdown file (`the corresponding file.md`) listed under *untracked* or *modified files*.

### 4. **Track and Add the file**:

```bash
git remote add origin <rep link>
git add thecorrespondingfile.md
```

To add whole folder 

```bash
git add .
```

### 5. **Commit the file**:

```bash
git commit -m "Add notes"
```

### 6. **Push to GitHub**:

```bash
git push origin main
```


### 6. **Sometimes you might need to remove when it didnot commit the rep**:

```bash
git remote remove origin
```




###  Final Result

Your file will now be live.





