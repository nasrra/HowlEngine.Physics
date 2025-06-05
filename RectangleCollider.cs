using System.Numerics;
using HowlEngine.Collections.Shapes;

namespace HowlEngine.Physics;

public struct RectangleCollider{
    private Rectangle _rectangle;
    
    /// <summary>
    /// Gets and Sets the xy-position of the collider.
    /// </summary>
    public Vector2 Position{
        get => new Vector2(_rectangle.X, _rectangle.Y);
        set {
            _rectangle.X = value.X;
            _rectangle.Y = value.Y;
        }
    }

    /// <summary>
    /// Gets and Sets the width the of the collider.
    /// </summary>
    public float Width {
        get => _rectangle.Width;
        set => _rectangle.Width = value;
    }

    /// <summary>
    /// Gets and Sets the height of the collider.
    /// </summary>
    public float Height {
        get => _rectangle.Height;
        set => _rectangle.Height = value;
    }

    /// <summary>
    /// Gets and Sets the x-position of the collider.
    /// </summary>
    public float X {
        get => _rectangle.X;
        set => _rectangle.X = value;
    }

    /// <summary>
    /// Gets and Sets the y_posiiton of the collider.
    /// </summary>
    public float Y {
        get => _rectangle.Y;
        set => _rectangle.Y = value;
    }

    /// <summary>
    /// Gets the x-coordinate of the left edge of this Collider.
    /// </summary>
    public float Left => _rectangle.Left;

    /// <summary>
    /// Gets the x-coordinate of the right edge of this Collider.
    /// </summary>
    public float Right => _rectangle.Right;

    /// <summary>
    /// Gets the y-coordinate of the top edge of this Collider.
    /// </summary>
    public float Top => _rectangle.Top;

    /// <summary>
    /// Gets the y-coordinate of the bottom edge of this Collider.
    /// </summary>
    public float Bottom => _rectangle.Bottom;


    /// <summary>
    /// Creates a new instance of RectangleCollider, with the specified position, width, and height.
    /// </summary>
    /// <param name="x">The x-coordinate of the top-left corner of the collider.</param>
    /// <param name="y">The y-coordinate of the top-left corner of the collider.</param>
    /// <param name="width">The width of the collider.</param>
    /// <param name="height">The height of the collider.</param>
    public RectangleCollider(float x, float y, float width, float height){
        _rectangle = new Rectangle(x,y,width,height);
    }
}