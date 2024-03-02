from .StereoFractAnalyzer import StereoFractAnalyzer
import argparse

def main():
    parser = argparse.ArgumentParser(description='Calculate the fractal dimension of a 3D surface or a 2D image.')
    parser.add_argument('--stl', type=str, help='Path to the STL file')
    parser.add_argument('--img', type=str, help='Path to the image file')
    args = parser.parse_args()
    if args.stl is None and args.img is None:
        print('Please provide a valid path to the STL file or the image file. -h or --help for help.')
        exit()

    elif args.stl is not None and args.img is None:
        fractal = StereoFractAnalyzer(args.stl)

        try:
            fractal.calculate_fractal_dimension()
        except Exception as e:
            print(f"An error occurred during fractal dimension calculation: {e}")

    elif args.img is not None and args.stl is None:
        fractal = StereoFractAnalyzer(img_path=args.img)
        fractal.get_image_fractal_dimension()
    else:
        fractal = StereoFractAnalyzer(args.stl, args.img)
        try:
            fractal.calculate_fractal_dimension()
        except Exception as e:
            print(f"An error occurred during fractal dimension calculation: {e}")

