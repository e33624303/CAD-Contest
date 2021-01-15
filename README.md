# CAD Contest

2018 CAD Contest - Problem B

Spec: http://iccad-contest.org/2018/Problem_B/2018ICCADContest_ProblemB.pdf

Contest Team: cada048 - Ming-Jie Fong, Ching-Hsi Chen, Wei-Ren Lai, He-Cheng Tsai

Advisor: Professor Yih-Lang Li

Result: Top X Team in the contest

## Usage :
```bash
./cada048 <input_file_name> <output_file_name>
```

## Concept :

A* search + Line-probe algorithm

Line-probe algroithm :

Repeatedly extend lines based on the existing lines & points until the lines extended from start intersect with the lines extended from the target

  Pros: It will be faster than traditional Maze routing Algorithm which is based on BFS searching. Also, there will be fewer corner, the lines will be extended until they encounter the boundary or the obstacles.
  
<img width="423" alt="Screen Shot 2021-01-15 at 2 48 29 PM" src="https://user-images.githubusercontent.com/12776044/104777197-e5cef080-5740-11eb-8ed1-2d1f10afb8b9.png">

A* search :

Weighted depth-first search

<img width="834" alt="Screen Shot 2021-01-15 at 2 49 27 PM" src="https://user-images.githubusercontent.com/12776044/104777198-e6678700-5740-11eb-9aea-82534532b6a3.png">

## Real Testcases:

<img width="403" alt="Screen Shot 2021-01-15 at 2 51 02 PM" src="https://user-images.githubusercontent.com/12776044/104777378-2c244f80-5741-11eb-826c-4f40a5664abe.png">

<img width="824" alt="Screen Shot 2021-01-15 at 2 51 11 PM" src="https://user-images.githubusercontent.com/12776044/104777383-2cbce600-5741-11eb-800c-ce620de736ce.png">

