#ifndef morph_H
#define morph_H

#include "opencv2/core/core.hpp"

namespace vision_commons {
	class Contour {
		public:
			static std::vector<std::vector<cv::Point> > getBestX(
				cv::Mat raw,
				int x
				);
			static std::vector<std::vector<cv::Point> > getBestXConvexHulled(
				cv::Mat raw,
				int x
				);
	};
}

#endif
