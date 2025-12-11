_base_ = [
    '../_base_/datasets/school_seg.py', '../_base_/models/frnet.py',
    '../_base_/schedules/onecycle-50k.py', '../_base_/default_runtime.py'
]
custom_imports = dict(
    imports=['frnet.datasets', 'frnet.datasets.transforms', 'frnet.models'],
    allow_failed_imports=False)

model = dict(
    data_preprocessor=dict(
        H=32, W=1024, fov_up=23.0, fov_down=-23.0, ignore_index=2),
    backbone=dict(output_shape=(32, 1024)),
    decode_head=dict(num_classes=3, ignore_index=2),
    auxiliary_head=[
        dict(
            type='FrustumHead',
            channels=128,
            num_classes=6,
            dropout_ratio=0,
            loss_ce=dict(
                type='mmdet.CrossEntropyLoss',
                use_sigmoid=False,
                class_weight=None,
                loss_weight=1.0),
            loss_lovasz=dict(
                type='LovaszLoss', loss_weight=1.5, reduction='none'),
            loss_boundary=dict(type='BoundaryLoss', loss_weight=1.0),
            conv_seg_kernel_size=1,
            ignore_index=2),
        dict(
            type='FrustumHead',
            channels=128,
            num_classes=6,
            dropout_ratio=0,
            loss_ce=dict(
                type='mmdet.CrossEntropyLoss',
                use_sigmoid=False,
                class_weight=None,
                loss_weight=1.0),
            loss_lovasz=dict(
                type='LovaszLoss', loss_weight=1.5, reduction='none'),
            loss_boundary=dict(type='BoundaryLoss', loss_weight=1.0),
            conv_seg_kernel_size=1,
            ignore_index=2,
            indices=2),
        dict(
            type='FrustumHead',
            channels=128,
            num_classes=6,
            dropout_ratio=0,
            loss_ce=dict(
                type='mmdet.CrossEntropyLoss',
                use_sigmoid=False,
                class_weight=None,
                loss_weight=1.0),
            loss_lovasz=dict(
                type='LovaszLoss', loss_weight=1.5, reduction='none'),
            loss_boundary=dict(type='BoundaryLoss', loss_weight=1.0),
            conv_seg_kernel_size=1,
            ignore_index=2,
            indices=3),
        dict(
            type='FrustumHead',
            channels=128,
            num_classes=6,
            dropout_ratio=0,
            loss_ce=dict(
                type='mmdet.CrossEntropyLoss',
                use_sigmoid=False,
                class_weight=None,
                loss_weight=1.0),
            loss_lovasz=dict(
                type='LovaszLoss', loss_weight=1.5, reduction='none'),
            loss_boundary=dict(type='BoundaryLoss', loss_weight=1.0),
            conv_seg_kernel_size=1,
            ignore_index=2,
            indices=4),
    ])
