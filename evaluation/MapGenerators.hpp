#ifndef MAPGENERATORS_HPP
#define MAPGENERATORS_HPP

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace map_generator {
template <class Pt>
class Maze {
public:
    Maze(int width, cv::Mat orig_map)
        : width(std::max(3, width)), orig_map(orig_map), border(1)
    {
        w = orig_map.rows / width;
        h = orig_map.cols / width;
        N = w * h;

        border = std::max(1, width/4);

        init();
    }

    virtual void run(const Pt& start, const Pt& goal) {
        cv::Point s(start.x / width, start.y / width);
        cv::Point g(goal.x / width, goal.y / width);

        expanded = 0;

        std::stack<cv::Point> open;
        cv::Point current = s;
        mark(current);

        while(expanded < N) {
            if(hasUnvisitedNeighbor(current)) {
                cv::Point next = getRandomUnvisitedNeighborCell(current);
                assert(!visited[IDX(next.x, next.y)]);

                open.push(current);

                removeWallsBetween(current, next);

                current = next;
                mark(current);

                //                cv::imshow("debug", orig_map);
                //                cv::waitKey(25);

            } else if(!open.empty()) {
                current = open.top();
                open.pop();

            } else {
                std::cout << "find random" << std::endl;
                current = getRandomUnvisitedCell();
                mark(current);
            }
        }
        std::cout << "done" << std::endl;

        assert(expanded == N);
    }

    virtual ~Maze() {
        delete[] visited;
    }

protected:
    void init() {
        memset(dx__, 0, 4 * sizeof(int));
        memset(dy__, 0, 4 * sizeof(int));
        dx__[0] = -1; dx__[2] = 1;
        dy__[1] = -1; dy__[3] = 1;

        visited = new int[N];
        memset(visited, true, (N) * sizeof(int));

        for(int y=0; y<w; y++) {
            for(int x=0; x<h; x++) {
                visited[IDX(x,y)] = false;
            }
        }
    }

    void mark(const cv::Point& pt) {
        visited[IDX(pt.x, pt.y)] = true;
        expanded++;
    }

    bool hasUnvisitedNeighbor(const cv::Point& pt) {
        for(int n = 0; n < 4; ++n) {
            int idx = IDX(pt.x + dx__[n], pt.y+dy__[n]);
            if(idx >= 0 && !visited[idx]) {
                return true;
            }
        }

        return false;
    }

    cv::Point getRandomUnvisitedCell() {
        while(true) {
            int x = rand() % h;
            int y = rand() % w;

            while(visited[IDX(x, y)]) {
                ++x;
                if(x == h) {
                    x = 0;
                    ++y;
                }
                if(y == w) {
                    y = 0;
                }
            }

            return cv::Point(x,y);
        }

        assert(false);
    }

    cv::Point getRandomUnvisitedNeighborCell(const cv::Point& pt) {
        assert(hasUnvisitedNeighbor(pt));

        while(true) {
            int n = rand() % 4;
            int dx = dx__[n];
            int dy = dy__[n];

            int idx = IDX(pt.x + dx, pt.y+dy);
            if(idx >= 0 && !visited[idx]) {
                cv::Point next(pt.x + dx, pt.y + dy);
                if(next.x < 0 || next.y < 0 || next.x == h || next.y == w) {
                    continue;
                } else {
                    return next;
                }
            }
        }

        assert(false);
    }

    void removeWallsBetween(const cv::Point& current, const cv::Point& next) {
        cv::Rect r1 = rectFor(current);
        cv::Rect r2 = rectFor(next);
        cv::Rect middle(0.5 * (r1.tl() + r2.tl()), r1.size());

        cv::rectangle(orig_map, r1, cv::Scalar::all(255), CV_FILLED);
        cv::rectangle(orig_map, middle, cv::Scalar::all(255), CV_FILLED);
        cv::rectangle(orig_map, r2, cv::Scalar::all(255), CV_FILLED);
    }

    inline int IDX(int x, int y) {
        if(x < 0 || y < 0 || x == h || y == w) {
            return -1;
        }
        return y*h + x;
    }

    inline void free(const cv::Point& pt) {
        free(pt.x, pt.y);
    }

    inline void free(int x, int y) {
        cv::rectangle(orig_map, rectFor(x, y), cv::Scalar::all(255), CV_FILLED);
    }

    inline cv::Rect rectFor(const cv::Point& pt) {
        return rectFor(pt.x, pt.y);
    }

    inline cv::Rect rectFor(int x, int y){
        return cv::Rect(x*width+border/2,y*width+border/2,width-border,width-border);
    }

protected:
    int dx__[4];
    int dy__[4];

    int width;
    cv::Mat orig_map;
    int border;

    int N;
    int w;
    int h;

    int* visited;
    int expanded;
};



template <class Pt>
class HilbertCurve : public Maze<Pt> {
    using Maze<Pt>::width;
    using Maze<Pt>::removeWallsBetween;

public:
    HilbertCurve(int width, cv::Mat orig_map)
        : Maze<Pt>(width, orig_map)
    {
    }

    void run(const Pt& start, const Pt& goal) {
        cv::Point s(start.x / width, start.y / width);
        cv::Point g(goal.x / width, goal.y / width);

        current = cv::Point(0, 0);

        int level = 8;

        hilbert_level(level);

        std::cout << "done" << std::endl;
    }

    enum DIR {
      UP,
      LEFT,
      DOWN,
      RIGHT
    };

    void move(DIR d)
    {
        cv::Point next = current;

        switch(d) {
        case LEFT:
            next.x -= 1;
          break;
        case RIGHT:
            next.x += 1;
          break;
        case UP:
            next.y -= 1;
          break;
        case DOWN:
            next.y += 1;
          break;
        }

        std::cout << next << std::endl;

        removeWallsBetween(current, next);
        current = next;
    }

    void hilbert_level(int level,int direction=UP)
    {
      if (level==1) {
        switch (direction) {
        case LEFT:
          move(RIGHT);      /* move() could draw a line in... */
          move(DOWN);       /* ...the indicated direction */
          move(LEFT);
          break;
        case RIGHT:
          move(LEFT);
          move(UP);
          move(RIGHT);
          break;
        case UP:
          move(DOWN);
          move(RIGHT);
          move(UP);
          break;
        case DOWN:
          move(UP);
          move(LEFT);
          move(DOWN);
          break;
        } /* switch */
      } else {
        switch (direction) {
        case LEFT:
          hilbert_level(level-1,UP);
          move(RIGHT);
          hilbert_level(level-1,LEFT);
          move(DOWN);
          hilbert_level(level-1,LEFT);
          move(LEFT);
          hilbert_level(level-1,DOWN);
          break;
        case RIGHT:
          hilbert_level(level-1,DOWN);
          move(LEFT);
          hilbert_level(level-1,RIGHT);
          move(UP);
          hilbert_level(level-1,RIGHT);
          move(RIGHT);
          hilbert_level(level-1,UP);
          break;
        case UP:
          hilbert_level(level-1,LEFT);
          move(DOWN);
          hilbert_level(level-1,UP);
          move(RIGHT);
          hilbert_level(level-1,UP);
          move(UP);
          hilbert_level(level-1,RIGHT);
          break;
        case DOWN:
          hilbert_level(level-1,RIGHT);
          move(UP);
          hilbert_level(level-1,DOWN);
          move(LEFT);
          hilbert_level(level-1,DOWN);
          move(DOWN);
          hilbert_level(level-1,LEFT);
          break;
        } /* switch */
      } /* if */
    }

protected:
    cv::Point current;

    double scale;
};
}
#endif // MAPGENERATORS_HPP
