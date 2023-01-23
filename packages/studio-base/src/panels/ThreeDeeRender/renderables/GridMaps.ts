// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

// The grid_map message specification can be found here:
// > https://github.com/ANYbotics/grid_map/tree/ros2/grid_map_msgs
//
// Thankfully, it's not too complicated. The message describes multiple
// layers of a single map. The strangest thing about the map is its
// encoding. The data is packed Fortran / column-major, with the X
// axis pointing up and the Y axis pointing to the left. So, width is
// Y (inverted) and height is X. This convention can be seen in the
// metadata, which looks like this:
//
// msg.info.data.layers
//  - [0].label = "column_index"   // "X" in the frame defined in header
//  - [1].label = "row_index"      // "Z" in the frame defined in header
//
// A lot of this code was copied and modified from the OccupancyGrid
// renderable, and adapted to do the following:
//
//  1. Multiple layers and settings supported per grid_map message.
//  2. Added support to draw value as height along Z.
//  3. Added support to display with multiple color spectra.

import * as THREE from "three";

import { toNanoSec } from "@foxglove/rostime";
import { SettingsTreeAction, SettingsTreeFields } from "@foxglove/studio";
import type { RosValue } from "@foxglove/studio-base/players/types";

import { BaseUserData, Renderable } from "../Renderable";
import { Renderer } from "../Renderer";
import { PartialMessage, PartialMessageEvent, SceneExtension } from "../SceneExtension";
import { SettingsTreeEntry } from "../SettingsManager";
import { rgbaToCssString, SRGBToLinear, stringToRgba } from "../color";
import {
  normalizeHeader,
  normalizePose,
  normalizeInt8Array,
  normalizeTime,
} from "../normalizeMessages";
import { GridMap, GRID_MAP_DATATYPES } from "../ros";
import { BaseSettings } from "../settings";
import { topicIsConvertibleToSchema } from "../topicIsConvertibleToSchema";

export type LayerSettingsGridMap = BaseSettings & {
  frameLocked: boolean;
  minColor: string;
  maxColor: string;
  unknownColor: string;
  invalidColor: string;
};

const INVALID_GRID_MAP = "INVALID_GRID_MAP";

const DEFAULT_MIN_COLOR = { r: 1, g: 1, b: 1, a: 1 }; // white
const DEFAULT_MAX_COLOR = { r: 0, g: 0, b: 0, a: 1 }; // black
const DEFAULT_UNKNOWN_COLOR = { r: 0.5, g: 0.5, b: 0.5, a: 1 }; // gray
const DEFAULT_INVALID_COLOR = { r: 1, g: 0, b: 1, a: 1 }; // magenta

const DEFAULT_MIN_COLOR_STR = rgbaToCssString(DEFAULT_MIN_COLOR);
const DEFAULT_MAX_COLOR_STR = rgbaToCssString(DEFAULT_MAX_COLOR);
const DEFAULT_UNKNOWN_COLOR_STR = rgbaToCssString(DEFAULT_UNKNOWN_COLOR);
const DEFAULT_INVALID_COLOR_STR = rgbaToCssString(DEFAULT_INVALID_COLOR);

const DEFAULT_SETTINGS: LayerSettingsGridMap = {
  visible: false,
  frameLocked: false,
  minColor: DEFAULT_MIN_COLOR_STR,
  maxColor: DEFAULT_MAX_COLOR_STR,
  unknownColor: DEFAULT_UNKNOWN_COLOR_STR,
  invalidColor: DEFAULT_INVALID_COLOR_STR,
};

export type GridMapUserData = BaseUserData & {
  settings: LayerSettingsGridMap;
  topic: string;
  gridMap: GridMap;
  mesh: THREE.Mesh;
  texture: THREE.DataTexture;
  material: THREE.MeshBasicMaterial;
  pickingMaterial: THREE.ShaderMaterial;
};

export class GridMapRenderable extends Renderable<GridMapUserData> {
  public override dispose(): void {
    this.userData.texture.dispose();
    this.userData.material.dispose();
    this.userData.pickingMaterial.dispose();
  }

  public override details(): Record<string, RosValue> {
    return this.userData.gridMap;
  }
}

export class GridMaps extends SceneExtension<GridMapRenderable> {
  public constructor(renderer: Renderer) {
    super("foxglove.GridMaps", renderer);

    renderer.addSchemaSubscriptions(GRID_MAP_DATATYPES, this.handleOccupancyGrid);
  }

  public override settingsNodes(): SettingsTreeEntry[] {
    const configTopics = this.renderer.config.topics;
    const handler = this.handleSettingsAction;
    const entries: SettingsTreeEntry[] = [];
    for (const topic of this.renderer.topics ?? []) {
      if (!topicIsConvertibleToSchema(topic, GRID_MAP_DATATYPES)) {
        continue;
      }
      const config = (configTopics[topic.name] ?? {}) as Partial<LayerSettingsGridMap>;

      // prettier-ignore
      const fields: SettingsTreeFields = {
        minColor: { label: "Min Color", input: "rgba", value: config.minColor ?? DEFAULT_MIN_COLOR_STR },
        maxColor: { label: "Max Color", input: "rgba", value: config.maxColor ?? DEFAULT_MAX_COLOR_STR },
        unknownColor: { label: "Unknown Color", input: "rgba", value: config.unknownColor ?? DEFAULT_UNKNOWN_COLOR_STR },
        invalidColor: { label: "Invalid Color", input: "rgba", value: config.invalidColor ?? DEFAULT_INVALID_COLOR_STR },
        frameLocked: { label: "Frame Lock", input: "boolean", value: config.frameLocked ?? false },
      };

      entries.push({
        path: ["topics", topic.name],
        node: {
          label: topic.name,
          icon: "Cells",
          fields,
          visible: config.visible ?? DEFAULT_SETTINGS.visible,
          order: topic.name.toLocaleLowerCase(),
          handler,
        },
      });
    }
    return entries;
  }

  public override handleSettingsAction = (action: SettingsTreeAction): void => {
    const path = action.payload.path;
    if (action.action !== "update" || path.length !== 3) {
      return;
    }

    this.saveSetting(path, action.payload.value);

    // Update the renderable
    const topicName = path[1]!;
    const renderable = this.renderables.get(topicName);
    if (renderable) {
      const prevTransparent = occupancyGridHasTransparency(renderable.userData.settings);
      const settings = this.renderer.config.topics[topicName] as
        | Partial<LayerSettingsGridMap>
        | undefined;
      renderable.userData.settings = { ...DEFAULT_SETTINGS, ...settings };

      // Check if the transparency changed and we need to create a new material
      const newTransparent = occupancyGridHasTransparency(renderable.userData.settings);
      if (prevTransparent !== newTransparent) {
        renderable.userData.material.transparent = newTransparent;
        renderable.userData.material.depthWrite = !newTransparent;
        renderable.userData.material.needsUpdate = true;
      }

      this._updateGridMapRenderable(
        renderable,
        renderable.userData.gridMap,
        renderable.userData.receiveTime,
      );
    }
  };

  private handleOccupancyGrid = (messageEvent: PartialMessageEvent<GridMap>): void => {
    const topic = messageEvent.topic;
    const gridMap = normalizeGridMap(messageEvent.message);
    const receiveTime = toNanoSec(messageEvent.receiveTime);

    let renderable = this.renderables.get(topic);
    if (!renderable) {
      // Set the initial settings from default values merged with any user settings
      const userSettings = this.renderer.config.topics[topic] as
        | Partial<LayerSettingsGridMap>
        | undefined;
      const settings = { ...DEFAULT_SETTINGS, ...userSettings };

      const texture = createTexture(gridMap);
      const geometry = this.renderer.sharedGeometry.getGeometry(
        this.constructor.name,
        createGeometry,
      );
      const mesh = createMesh(topic, geometry, texture, settings);
      const material = mesh.material as THREE.MeshBasicMaterial;
      const pickingMaterial = mesh.userData.pickingMaterial as THREE.ShaderMaterial;

      // Create the renderable
      renderable = new GridMapRenderable(topic, this.renderer, {
        receiveTime,
        messageTime: toNanoSec(gridMap.header.stamp),
        frameId: this.renderer.normalizeFrameId(gridMap.header.frame_id),
        pose: gridMap.info.pose,
        settingsPath: ["topics", topic],
        settings,
        topic,
        gridMap,
        mesh,
        texture,
        material,
        pickingMaterial,
      });
      renderable.add(mesh);

      this.add(renderable);
      this.renderables.set(topic, renderable);
    }

    this._updateGridMapRenderable(renderable, gridMap, receiveTime);
  };

  private _updateGridMapRenderable(
    renderable: GridMapRenderable,
    gridMap: GridMap,
    receiveTime: bigint,
  ): void {
    renderable.userData.gridMap = gridMap;
    renderable.userData.pose = gridMap.info.pose;
    renderable.userData.receiveTime = receiveTime;
    renderable.userData.messageTime = toNanoSec(gridMap.header.stamp);
    renderable.userData.frameId = this.renderer.normalizeFrameId(gridMap.header.frame_id);

    // Check that every layer matches the size of the grid in the .info field.
    const resolution = gridMap.info.resolution;
    const height = gridMap.info.length_x / resolution;
    const width = gridMap.info.length_y / resolution;
    const size = width * height;
    for (const layer of gridMap.data ?? []) {
        if (layer.layout.dim.length < 2) {
            const message = `GridMap layer is has the wrong number of dimensions`;
            invalidGridMapError(this.renderer, renderable, message);
            return;
        }
        let layer_size = 0;
        for (const dim of layer.layout.dim ?? []) {
            layer_size = layer_size + dim.size;
        }
        if (layer_size !== size) {
            const message = `GridMap layer is not equal to width ${width} * height ${height}`;
            invalidGridMapError(this.renderer, renderable, message);
            return;
        }
    }

    // The image dimensions changed, regenerate the texture
    let texture = renderable.userData.texture;
    if (width !== texture.image.width || height !== texture.image.height) {
      texture.dispose();
      texture = createTexture(gridMap);
      renderable.userData.texture = texture;
      renderable.userData.material.map = texture;
    }

    // Update the occupancy grid texture
    updateTexture(texture, gridMap, renderable.userData.settings);

    renderable.scale.set(resolution * width, resolution * height, 1);
  }
}
function createGeometry(): THREE.PlaneGeometry {
  const geometry = new THREE.PlaneGeometry(1, 1, 1, 1);
  geometry.translate(0.5, 0.5, 0);
  geometry.computeBoundingSphere();
  return geometry;
}
function invalidGridMapError(
  renderer: Renderer,
  renderable: GridMapRenderable,
  message: string,
): void {
  renderer.settings.errors.addToTopic(renderable.userData.topic, INVALID_GRID_MAP, message);
}

function createTexture(gridMap: GridMap): THREE.DataTexture {
  const height = gridMap.info.length_x / gridMap.info.resolution;
  const width = gridMap.info.length_y / gridMap.info.resolution;
  const size = width * height;
  const rgba = new Uint8ClampedArray(size * 4);
  const texture = new THREE.DataTexture(
    rgba,
    width,
    height,
    THREE.RGBAFormat,
    THREE.UnsignedByteType,
    THREE.UVMapping,
    THREE.ClampToEdgeWrapping,
    THREE.ClampToEdgeWrapping,
    THREE.NearestFilter,
    THREE.LinearFilter,
    1,
    THREE.LinearEncoding, // GridMap carries linear grayscale values, not sRGB
  );
  texture.generateMipmaps = false;
  return texture;
}

function createMesh(
  topic: string,
  geometry: THREE.PlaneGeometry,
  texture: THREE.DataTexture,
  settings: LayerSettingsGridMap,
): THREE.Mesh {
  // Create the texture, material, and mesh
  const pickingMaterial = createPickingMaterial(texture);
  const material = createMaterial(texture, topic, settings);
  const mesh = new THREE.Mesh(geometry, material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  // This overrides the picking material used for `mesh`. See Picker.ts
  mesh.userData.pickingMaterial = pickingMaterial;
  return mesh;
}

const tempUnknownColor = { r: 0, g: 0, b: 0, a: 0 };
const tempInvalidColor = { r: 0, g: 0, b: 0, a: 0 };
const tempMinColor = { r: 0, g: 0, b: 0, a: 0 };
const tempMaxColor = { r: 0, g: 0, b: 0, a: 0 };

function updateTexture(
  texture: THREE.DataTexture,
  gridMap: GridMap,
  settings: LayerSettingsGridMap,
): void {
  const height = gridMap.info.length_x / gridMap.info.resolution;
  const width = gridMap.info.length_y / gridMap.info.resolution;
  const size = width * height;
  const rgba = texture.image.data;
  stringToRgba(tempMinColor, settings.minColor);
  stringToRgba(tempMaxColor, settings.maxColor);
  stringToRgba(tempUnknownColor, settings.unknownColor);
  stringToRgba(tempInvalidColor, settings.invalidColor);

  srgbToLinearUint8(tempMinColor);
  srgbToLinearUint8(tempMaxColor);
  srgbToLinearUint8(tempUnknownColor);
  srgbToLinearUint8(tempInvalidColor);

  const data = gridMap.data;
  for (let i = 0; i < size; i++) {
    const value = data[i]! | 0;
    const offset = i * 4;
    if (value === -1) {
      // Unknown (-1)
      rgba[offset + 0] = tempUnknownColor.r;
      rgba[offset + 1] = tempUnknownColor.g;
      rgba[offset + 2] = tempUnknownColor.b;
      rgba[offset + 3] = tempUnknownColor.a;
    } else if (value >= 0 && value <= 100) {
      // Valid [0-100]
      const t = value / 100;
      if (t === 1) {
        rgba[offset + 0] = 0;
        rgba[offset + 1] = 0;
        rgba[offset + 2] = 0;
        rgba[offset + 3] = 0;
      } else {
        rgba[offset + 0] = tempMinColor.r + (tempMaxColor.r - tempMinColor.r) * t;
        rgba[offset + 1] = tempMinColor.g + (tempMaxColor.g - tempMinColor.g) * t;
        rgba[offset + 2] = tempMinColor.b + (tempMaxColor.b - tempMinColor.b) * t;
        rgba[offset + 3] = tempMinColor.a + (tempMaxColor.a - tempMinColor.a) * t;
      }
    } else {
      // Invalid (< -1 or > 100)
      rgba[offset + 0] = tempInvalidColor.r;
      rgba[offset + 1] = tempInvalidColor.g;
      rgba[offset + 2] = tempInvalidColor.b;
      rgba[offset + 3] = tempInvalidColor.a;
    }
  }

  texture.needsUpdate = true;
}

function createMaterial(
  texture: THREE.DataTexture,
  topic: string,
  settings: LayerSettingsGridMap,
): THREE.MeshBasicMaterial {
  const transparent = occupancyGridHasTransparency(settings);
  return new THREE.MeshBasicMaterial({
    name: `${topic}:Material`,
    // Enable alpha clipping. Fully transparent (alpha=0) pixels are skipped
    // even when transparency is disabled
    alphaTest: 1e-4,
    depthWrite: !transparent,
    map: texture,
    side: THREE.DoubleSide,
    transparent,
  });
}

function createPickingMaterial(texture: THREE.DataTexture): THREE.ShaderMaterial {
  return new THREE.ShaderMaterial({
    vertexShader: /* glsl */ `
      varying vec2 vUv;
      void main() {
        vUv = uv;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
      }
    `,
    fragmentShader: /* glsl */ `
      uniform sampler2D map;
      uniform vec4 objectId;
      varying vec2 vUv;
      void main() {
        vec4 color = texture2D(map, vUv);
        if (color.a == 0.0) {
          discard;
        }
        gl_FragColor = objectId;
      }
    `,
    side: THREE.DoubleSide,
    uniforms: { map: { value: texture }, objectId: { value: [NaN, NaN, NaN, NaN] } },
  });
}

function occupancyGridHasTransparency(settings: LayerSettingsGridMap): boolean {
  stringToRgba(tempMinColor, settings.minColor);
  stringToRgba(tempMaxColor, settings.maxColor);
  stringToRgba(tempUnknownColor, settings.unknownColor);
  stringToRgba(tempInvalidColor, settings.invalidColor);
  return (
    tempMinColor.a < 1 || tempMaxColor.a < 1 || tempInvalidColor.a < 1 || tempUnknownColor.a < 1
  );
}

function srgbToLinearUint8(color: ColorRGBA): void {
  color.r = Math.trunc(SRGBToLinear(color.r) * 255);
  color.g = Math.trunc(SRGBToLinear(color.g) * 255);
  color.b = Math.trunc(SRGBToLinear(color.b) * 255);
  color.a = Math.trunc(color.a * 255);
}

function normalizeGridMap(message: PartialMessage<GridMap>): GridMap {
  const info = message.info ?? {};

  return {
    header: normalizeHeader(message.header),
    info: {
      resolution: info.resolution ?? 0,
      length_x: info.length_x ?? 0,
      length_y: info.length_y ?? 0,
      pose: normalizePose(info.pose),
    },
    data: message.data.map(normalizeFloat32MultiArray)
  };
}
