#ifndef POINTCLASSIFICATION_H
#define POINTCLASSIFICATION_H

struct PointClassification {
    bool traversable_by_range;
    bool traversable_by_intensity;

    bool operator==(PointClassification p) {
        return (this->traversable_by_intensity == p.traversable_by_intensity)
                && (this->traversable_by_range == p.traversable_by_range);
    }

    bool operator!=(PointClassification p) {
        return !operator ==(p);
    }

    bool isTraversable() {
        return traversable_by_intensity && traversable_by_range;
    }
};

#endif // POINTCLASSIFICATION_H
