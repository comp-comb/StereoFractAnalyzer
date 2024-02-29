import pytest
from StereoFractAnalyzer import StereoFractAnalyzer
import numpy as np
from PIL import Image
import os



# Fixture for creating a common analyzer object
@pytest.fixture
def sample_analyzer():

    return StereoFractAnalyzer(stl_path="./test.stl", img_path="./test.png")

def test_transform(sample_analyzer):
    original_position = np.array(sample_analyzer.STL.get_center())
    translation_vector = np.array((1, 1, 1))
    sample_analyzer.transform(translation=translation_vector)
    new_position = np.array(sample_analyzer.STL.get_center())
    np.testing.assert_array_equal(new_position, original_position + translation_vector)

def test_get_voxel(sample_analyzer):
    voxel_grid = sample_analyzer.get_voxel(size=1)
    # Assuming some expected outcome, e.g., voxel grid is not empty
    assert len(voxel_grid) > 0, "Voxel grid should not be empty"

def test_calculate_intersection_point():
    analyzer = StereoFractAnalyzer()  # Without specific paths for simplicity
    p1 = np.array([0, 0, 0])
    p2 = np.array([1, 1, 1])
    z_height = 0.5
    expected_intersection = np.array([0.5, 0.5, 0.5])
    intersection = analyzer.calculate_intersection_point(p1, p2, z_height)
    np.testing.assert_array_almost_equal(intersection, expected_intersection)


@pytest.fixture(scope="session")
def test_image_path():
    img = Image.new('RGB', (100, 100), color = 'black')
    img_path = "test_image.png"
    img.save(img_path)
    yield img_path
    # Cleanup
    os.remove(img_path)

def test_calculate_slice(sample_analyzer):
    z_height = 0.5
    slice_lines = sample_analyzer.calculate_slice(sample_analyzer.STL, z_height)
    assert isinstance(slice_lines, np.ndarray), "Expected slice lines to be a numpy array"
    assert slice_lines.ndim == 3, "Slice lines array should have 3 dimensions"
    assert slice_lines.shape[1] == 2, "Each slice line should consist of 2 points"

def test_get_image_fractal_dimension(sample_analyzer, test_image_path):
    fractal_dimension = sample_analyzer.get_image_fractal_dimension(img_path=test_image_path, plot=0)
    assert isinstance(fractal_dimension, float), "Fractal dimension should be a float"
    assert fractal_dimension > 0, "Fractal dimension should be positive"



@pytest.mark.parametrize("z_height, expected_output", [
    (0.5, True),
    (-1, False),  # Assuming z_height < 0 would not intersect with the mesh
    (100, False), # Assuming z_height well above the mesh would not intersect
])
def test_calculate_slice_validity(sample_analyzer, z_height, expected_output):
    slice_lines = sample_analyzer.calculate_slice(sample_analyzer.STL, z_height)
    valid_output = len(slice_lines) > 0
    assert valid_output == expected_output, f"Expected slice validity at z={z_height} to be {expected_output}"

